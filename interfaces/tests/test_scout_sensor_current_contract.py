"""Public-contract tests for Scout sensor geometry, health, and current inputs."""

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
PROTO_V1 = PROTO_ROOT / "underwater" / "contracts" / "v1"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
CONTRACT_PATH = INTERFACES / "SCOUT_SENSOR_AND_CURRENT.md"
HASHING_PATH = INTERFACES / "HASHING.md"
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"


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


def rotate_vector_by_quaternion(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a sensor-frame vector into base_link using an x/y/z/w quaternion."""
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def sensor_range_contains(distance_m: float, field_of_view: object) -> bool:
    return field_of_view.minimum_range_m <= distance_m <= field_of_view.maximum_range_m


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


def _finite_present(message: object, fields: tuple[str, ...]) -> bool:
    return all(message.HasField(field) and math.isfinite(getattr(message, field)) for field in fields)


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
        and "scout_sensor_and_current_v1" in manifest["supported_features"]
    )


def validate_sensor_geometry(
    geometry: object,
    sensing: object,
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
    if _has_unknown_fields(geometry):
        raise ValueError("unknown field in sensor geometry")
    limits = profile["interface_limits"]
    if (
        len(geometry.SerializeToString(deterministic=True))
        > limits["maximum_sensor_geometry_bytes"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(geometry)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: sensor geometry")
    if not _valid_header(
        geometry.header,
        common,
        stream_id=common.STREAM_SCOUT_SENSOR_GEOMETRY,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("invalid sensor geometry header, clock, or manifest")
    if (
        not geometry.sensor_id
        or geometry.geometry_version == 0
        or geometry.sensor_type
        not in {
            sensing.SCOUT_SENSOR_MULTIBEAM_SONAR,
            sensing.SCOUT_SENSOR_FORWARD_LOOKING_SONAR,
            sensing.SCOUT_SENSOR_CAMERA,
        }
        or geometry.occlusion_policy
        != sensing.SENSOR_OCCLUSION_KNOWN_FREE_RAYCAST_REQUIRED
        or not geometry.operating_domain_id
        or geometry.operating_domain_id != profile["operating_domain_id"]
        or not geometry.device_serial_number
        or not geometry.calibration_dataset_id
        or not geometry.calibration_method_version
        or not geometry.HasField("calibrated_at_utc_ns")
        or geometry.calibrated_at_utc_ns <= 0
        or not geometry.HasField("production_approved")
    ):
        raise ValueError("uncalibrated or unsupported sensor geometry")
    if production_planning and (not profile["production"] or not geometry.production_approved):
        raise ValueError("NON_PRODUCTION sensor geometry")

    extrinsics = geometry.extrinsics
    extrinsic_fields = (
        "translation_x_m",
        "translation_y_m",
        "translation_z_m",
        "q_x",
        "q_y",
        "q_z",
        "q_w",
    )
    if (
        not _finite_present(extrinsics, extrinsic_fields)
        or extrinsics.body_frame_id != "base_link"
        or not extrinsics.sensor_frame_id
        or not math.isclose(
            sum(getattr(extrinsics, field) ** 2 for field in ("q_x", "q_y", "q_z", "q_w")),
            1.0,
            rel_tol=0.0,
            abs_tol=1e-9,
        )
    ):
        raise ValueError("invalid fixed sensor extrinsics or frame")

    fov = geometry.field_of_view
    fov_fields = (
        "horizontal_fov_rad",
        "vertical_fov_rad",
        "minimum_range_m",
        "maximum_range_m",
        "range_resolution_m",
        "horizontal_angular_resolution_rad",
        "vertical_angular_resolution_rad",
    )
    if (
        not _finite_present(fov, fov_fields)
        or not (0.0 < fov.horizontal_fov_rad <= 2.0 * math.pi)
        or not (0.0 < fov.vertical_fov_rad <= math.pi)
        or fov.minimum_range_m < 0.0
        or fov.maximum_range_m <= fov.minimum_range_m
        or fov.range_resolution_m <= 0.0
        or fov.horizontal_angular_resolution_rad <= 0.0
        or fov.vertical_angular_resolution_rad <= 0.0
    ):
        raise ValueError("invalid sensor FOV, range, or resolution")
    if (
        len(geometry.geometry_content_identity.sha256) != 32
        or not hmac.compare_digest(
            geometry.geometry_content_identity.sha256,
            canonical_business_identity(geometry, "geometry_content_identity"),
        )
    ):
        raise ValueError("FAULT_BUNDLE_INTEGRITY: sensor geometry content identity mismatch")


def validate_sensor_health(
    health: object,
    sensing: object,
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
    if _has_unknown_fields(health):
        raise ValueError("unknown field in sensor health")
    limits = profile["interface_limits"]
    if (
        len(health.SerializeToString(deterministic=True))
        > limits["maximum_sensor_health_bytes"]
        or len(health.active_fault_codes) > limits["maximum_diagnostics"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(health)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: sensor health")
    if not _valid_header(
        health.header,
        common,
        stream_id=common.STREAM_SCOUT_SENSOR_HEALTH,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("invalid sensor health header, clock, or manifest")
    reject_age_ns = profile["timing"]["scout_sensor_health_reject_ns"]
    if (
        not health.sensor_id
        or health.health_version == 0
        or not health.HasField("observed_at_monotonic_ns")
        or not health.HasField("valid_until_monotonic_ns")
        or health.observed_at_monotonic_ns < 0
        or health.valid_until_monotonic_ns < health.observed_at_monotonic_ns
        or now_monotonic_ns < health.observed_at_monotonic_ns
        or now_monotonic_ns > health.valid_until_monotonic_ns
        or now_monotonic_ns - health.observed_at_monotonic_ns > reject_age_ns
    ):
        raise ValueError("stale or invalid sensor health time")
    if health.health != sensing.SCOUT_SENSOR_HEALTH_NOMINAL:
        raise ValueError("sensor is not nominal for new planning")
    if health.active_fault_codes:
        raise ValueError("nominal sensor health cannot carry active fault codes")
    if (
        len(health.health_content_identity.sha256) != 32
        or not hmac.compare_digest(
            health.health_content_identity.sha256,
            canonical_business_identity(health, "health_content_identity"),
        )
    ):
        raise ValueError("FAULT_BUNDLE_INTEGRITY: sensor health content identity mismatch")


def validate_current_estimate(
    current: object,
    sensing: object,
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
    _validate_bound_profile(profile, profile_artifact_bytes, manifest)
    if _has_unknown_fields(current):
        raise ValueError("unknown field in current estimate")
    limits = profile["interface_limits"]
    if (
        len(current.SerializeToString(deterministic=True))
        > limits["maximum_current_estimate_bytes"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(current)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: current estimate")
    if not _valid_header(
        current.header,
        common,
        stream_id=common.STREAM_SCOUT_CURRENT_ESTIMATE,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("invalid current estimate header, clock, or manifest")
    if production_planning and not profile["production"]:
        raise ValueError("NON_PRODUCTION current profile")
    if (
        not current.current_model_id
        or current.current_model_version == 0
        or current.operating_domain_id != profile["operating_domain_id"]
        or not current.model_source_id
        or current.validity != sensing.CURRENT_ESTIMATE_VALID
        or not current.HasField("observed_at_monotonic_ns")
        or not current.HasField("valid_from_monotonic_ns")
        or not current.HasField("valid_until_monotonic_ns")
        or current.valid_from_monotonic_ns > current.observed_at_monotonic_ns
        or current.observed_at_monotonic_ns > current.valid_until_monotonic_ns
        or now_monotonic_ns < current.valid_from_monotonic_ns
        or now_monotonic_ns > current.valid_until_monotonic_ns
        or now_monotonic_ns - current.observed_at_monotonic_ns
        > profile["timing"]["scout_current_reject_ns"]
    ):
        raise ValueError("stale, invalid, or out-of-domain current estimate")
    region = current.applicable_region
    if (
        region.frame_id != "mission_enu"
        or len(region.xyz_m) != 6
        or not all(math.isfinite(value) for value in region.xyz_m)
        or not all(region.xyz_m[index] < region.xyz_m[index + 3] for index in range(3))
    ):
        raise ValueError("invalid current spatial domain or frame")
    if (
        current.reference_position.frame_id != "mission_enu"
        or not _finite_present(current.reference_position, ("x_m", "y_m", "z_m"))
        or not all(
            region.xyz_m[index]
            <= getattr(current.reference_position, ("x_m", "y_m", "z_m")[index])
            <= region.xyz_m[index + 3]
            for index in range(3)
        )
    ):
        raise ValueError("current reference position is out of domain")
    velocity = current.velocity_at_reference_mps
    component_error = current.component_error_bound_mps
    vector_fields = ("x_mps", "y_mps", "z_mps")
    if (
        velocity.frame_id != "mission_enu"
        or component_error.frame_id != "mission_enu"
        or not _finite_present(velocity, vector_fields)
        or not _finite_present(component_error, vector_fields)
        or any(getattr(component_error, field) < 0.0 for field in vector_fields)
        or not current.HasField("speed_error_bound_mps")
        or not math.isfinite(current.speed_error_bound_mps)
        or current.speed_error_bound_mps < 0.0
        or current.speed_error_bound_mps
        < math.sqrt(sum(getattr(component_error, field) ** 2 for field in vector_fields))
    ):
        raise ValueError("incomplete or invalid current error bound")
    if current.HasField("spatial_gradient"):
        gradient = current.spatial_gradient
        if (
            gradient.input_frame_id != "mission_enu"
            or gradient.output_frame_id != "mission_enu"
            or len(gradient.row_major_per_s) != 9
            or not all(math.isfinite(value) for value in gradient.row_major_per_s)
        ):
            raise ValueError("invalid optional current gradient")
    if (
        len(current.current_content_identity.sha256) != 32
        or not hmac.compare_digest(
            current.current_content_identity.sha256,
            canonical_business_identity(current, "current_content_identity"),
        )
    ):
        raise ValueError("FAULT_BUNDLE_INTEGRITY: current content identity mismatch")


def validate_sensor_context(
    geometry: object,
    health: object,
    sensing: object,
    common: object,
    *,
    clock_domain_id: str,
    now_monotonic_ns: int,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    profile: dict[str, object],
    profile_artifact_bytes: bytes,
    previous_dependencies: tuple[int, bytes, int, bytes] | None = None,
) -> tuple[tuple[int, bytes, int, bytes], bool]:
    validate_sensor_geometry(
        geometry,
        sensing,
        common,
        clock_domain_id=clock_domain_id,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
        profile=profile,
        profile_artifact_bytes=profile_artifact_bytes,
        production_planning=False,
    )
    validate_sensor_health(
        health,
        sensing,
        common,
        clock_domain_id=clock_domain_id,
        now_monotonic_ns=now_monotonic_ns,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
        profile=profile,
        profile_artifact_bytes=profile_artifact_bytes,
    )
    if geometry.sensor_id != health.sensor_id:
        raise ValueError("sensor geometry and health identity mismatch")
    incoming = (
        geometry.geometry_version,
        geometry.geometry_content_identity.sha256,
        health.health_version,
        health.health_content_identity.sha256,
    )
    if previous_dependencies is None:
        return incoming, False
    for current_version, current_identity, incoming_version, incoming_identity in (
        (
            previous_dependencies[0],
            previous_dependencies[1],
            incoming[0],
            incoming[1],
        ),
        (
            previous_dependencies[2],
            previous_dependencies[3],
            incoming[2],
            incoming[3],
        ),
    ):
        if incoming_version < current_version:
            raise ValueError("VERSION_INCOMPATIBLE: sensor dependency rollback")
        if incoming_version == current_version and not hmac.compare_digest(
            incoming_identity, current_identity
        ):
            raise ValueError("INPUT_INVALID: sensor dependency identity conflict")
    return incoming, incoming != previous_dependencies


class ScoutSensorAndCurrentContractTests(unittest.TestCase):
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
        cls.sensing = importlib.import_module("underwater.contracts.v1.sensing_pb2")
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
            producer_id="scout-input-authority",
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

    def _geometry(self) -> object:
        geometry = self.sensing.ScoutSensorGeometry(
            header=self._header(self.common.STREAM_SCOUT_SENSOR_GEOMETRY, 91),
            sensor_id="multibeam/bow",
            geometry_version=7,
            sensor_type=self.sensing.SCOUT_SENSOR_MULTIBEAM_SONAR,
            extrinsics=self.sensing.SensorExtrinsicsFlu(
                translation_x_m=0.42,
                translation_y_m=0.0,
                translation_z_m=-0.08,
                q_x=0.0,
                q_y=0.0,
                q_z=0.7071067811865475,
                q_w=0.7071067811865476,
                body_frame_id="base_link",
                sensor_frame_id="multibeam_link",
            ),
            field_of_view=self.sensing.SensorFieldOfView(
                horizontal_fov_rad=2.0943951023931953,
                vertical_fov_rad=0.5235987755982988,
                minimum_range_m=0.5,
                maximum_range_m=30.0,
                range_resolution_m=0.05,
                horizontal_angular_resolution_rad=0.008726646259971648,
                vertical_angular_resolution_rad=0.017453292519943295,
            ),
            occlusion_policy=self.sensing.SENSOR_OCCLUSION_KNOWN_FREE_RAYCAST_REQUIRED,
            operating_domain_id=self.profile["operating_domain_id"],
            device_serial_number="MBES-TEST-001",
            calibration_dataset_id="cal/scout-mbes/2026-08-bench",
            calibration_method_version="sensor-calibration/v1",
            calibrated_at_utc_ns=1_787_000_000_000_000_000,
            production_approved=False,
        )
        geometry.geometry_content_identity.sha256 = canonical_business_identity(
            geometry, "geometry_content_identity"
        )
        return geometry

    def _health(self) -> object:
        health = self.sensing.ScoutSensorHealthState(
            header=self._header(self.common.STREAM_SCOUT_SENSOR_HEALTH, 92),
            sensor_id="multibeam/bow",
            health_version=19,
            observed_at_monotonic_ns=1_000_000_000,
            valid_until_monotonic_ns=1_200_000_000,
            health=self.sensing.SCOUT_SENSOR_HEALTH_NOMINAL,
            active_fault_codes=[],
        )
        health.health_content_identity.sha256 = canonical_business_identity(
            health, "health_content_identity"
        )
        return health

    def _current(self) -> object:
        current = self.sensing.ScoutCurrentEstimate(
            header=self._header(self.common.STREAM_SCOUT_CURRENT_ESTIMATE, 93),
            current_model_id="local-current/scout-ekf",
            current_model_version=12,
            observed_at_monotonic_ns=1_000_000_000,
            valid_from_monotonic_ns=950_000_000,
            valid_until_monotonic_ns=1_200_000_000,
            applicable_region=self.sensing.CurrentRegion3dEnu(
                xyz_m=[-10.0, -20.0, -15.0, 40.0, 20.0, 0.0],
                frame_id="mission_enu",
            ),
            reference_position=self.sensing.Point3dEnu(
                x_m=5.0, y_m=0.0, z_m=-6.0, frame_id="mission_enu"
            ),
            velocity_at_reference_mps=self.sensing.Vector3dEnuMps(
                x_mps=0.25, y_mps=-0.10, z_mps=0.02, frame_id="mission_enu"
            ),
            component_error_bound_mps=self.sensing.Vector3dEnuMps(
                x_mps=0.05, y_mps=0.04, z_mps=0.03, frame_id="mission_enu"
            ),
            speed_error_bound_mps=0.08,
            spatial_gradient=self.sensing.CurrentGradient3d(
                row_major_per_s=[0.01, 0.0, 0.0, 0.0, -0.01, 0.0, 0.0, 0.0, 0.0],
                input_frame_id="mission_enu",
                output_frame_id="mission_enu",
            ),
            operating_domain_id=self.profile["operating_domain_id"],
            model_source_id="adcp-ekf/cal-2026-08",
            validity=self.sensing.CURRENT_ESTIMATE_VALID,
        )
        current.current_content_identity.sha256 = canonical_business_identity(
            current, "current_content_identity"
        )
        return current

    def _validate_geometry(self, geometry: object, *, production: bool = False) -> None:
        validate_sensor_geometry(
            geometry,
            self.sensing,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
            production_planning=production,
        )

    def _validate_health(self, health: object, *, now: int = 1_030_000_000) -> None:
        validate_sensor_health(
            health,
            self.sensing,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=now,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
        )

    def _validate_sensor_context(
        self,
        geometry: object,
        health: object,
        *,
        previous: tuple[int, bytes, int, bytes] | None = None,
    ) -> tuple[tuple[int, bytes, int, bytes], bool]:
        return validate_sensor_context(
            geometry,
            health,
            self.sensing,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=1_030_000_000,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
            previous_dependencies=previous,
        )

    def _validate_current(
        self, current: object, *, now: int = 1_030_000_000, production: bool = False
    ) -> None:
        validate_current_estimate(
            current,
            self.sensing,
            self.common,
            clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=now,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            profile=self.profile,
            profile_artifact_bytes=self.profile_artifact_bytes,
            production_planning=production,
        )

    def test_complete_sensor_and_current_inputs_round_trip_and_validate(self) -> None:
        geometry = self._geometry()
        health = self._health()
        current = self._current()

        self.assertEqual(type(geometry).FromString(geometry.SerializeToString()), geometry)
        self.assertEqual(type(health).FromString(health.SerializeToString()), health)
        self.assertEqual(type(current).FromString(current.SerializeToString()), current)
        self._validate_geometry(geometry)
        self._validate_health(health)
        self._validate_current(current)

    def test_geometry_enforces_fixed_mounting_visibility_and_calibration(self) -> None:
        invalid_cases = []
        wrong_frame = self._geometry()
        wrong_frame.extrinsics.body_frame_id = "base_link_frd"
        invalid_cases.append((wrong_frame, "extrinsics|frame"))
        bad_quaternion = self._geometry()
        bad_quaternion.extrinsics.q_w = 0.5
        invalid_cases.append((bad_quaternion, "extrinsics"))
        reversed_range = self._geometry()
        reversed_range.field_of_view.maximum_range_m = 0.25
        invalid_cases.append((reversed_range, "range"))
        uncalibrated = self._geometry()
        uncalibrated.calibration_dataset_id = ""
        invalid_cases.append((uncalibrated, "uncalibrated"))
        permissive_occlusion = self._geometry()
        permissive_occlusion.occlusion_policy = self.sensing.SENSOR_OCCLUSION_POLICY_UNSPECIFIED
        invalid_cases.append((permissive_occlusion, "unsupported"))

        for geometry, pattern in invalid_cases:
            geometry.geometry_content_identity.sha256 = canonical_business_identity(
                geometry, "geometry_content_identity"
            )
            with self.subTest(pattern=pattern), self.assertRaisesRegex(ValueError, pattern):
                self._validate_geometry(geometry)

        geometry = self._geometry()
        quaternion = (
            geometry.extrinsics.q_x,
            geometry.extrinsics.q_y,
            geometry.extrinsics.q_z,
            geometry.extrinsics.q_w,
        )
        rotated_boresight = rotate_vector_by_quaternion(quaternion, (1.0, 0.0, 0.0))
        for actual, expected in zip(rotated_boresight, (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(actual, expected)

        fov = geometry.field_of_view
        self.assertTrue(sensor_range_contains(fov.minimum_range_m, fov))
        self.assertTrue(sensor_range_contains(fov.maximum_range_m, fov))
        self.assertFalse(sensor_range_contains(fov.minimum_range_m - 1e-9, fov))
        self.assertFalse(sensor_range_contains(fov.maximum_range_m + 1e-9, fov))

    def test_geometry_and_health_versions_are_independent_plan_dependencies(self) -> None:
        geometry = self._geometry()
        health = self._health()
        original_dependencies, requires_revalidation = self._validate_sensor_context(
            geometry, health
        )
        self.assertFalse(requires_revalidation)

        changed_geometry = self._geometry()
        changed_geometry.geometry_version += 1
        changed_geometry.geometry_content_identity.sha256 = canonical_business_identity(
            changed_geometry, "geometry_content_identity"
        )
        geometry_dependencies, requires_revalidation = self._validate_sensor_context(
            changed_geometry, health, previous=original_dependencies
        )
        self.assertTrue(requires_revalidation)
        self.assertEqual(geometry_dependencies[2:], original_dependencies[2:])

        changed_health = self._health()
        changed_health.health_version += 1
        changed_health.health_content_identity.sha256 = canonical_business_identity(
            changed_health, "health_content_identity"
        )
        health_dependencies, requires_revalidation = self._validate_sensor_context(
            geometry, changed_health, previous=original_dependencies
        )
        self.assertTrue(requires_revalidation)
        self.assertEqual(health_dependencies[:2], original_dependencies[:2])

        mismatched = self._health()
        mismatched.sensor_id = "multibeam/stern"
        mismatched.health_content_identity.sha256 = canonical_business_identity(
            mismatched, "health_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "identity mismatch"):
            self._validate_sensor_context(geometry, mismatched)

        rollback = self._geometry()
        rollback.geometry_version -= 1
        rollback.geometry_content_identity.sha256 = canonical_business_identity(
            rollback, "geometry_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "VERSION_INCOMPATIBLE"):
            self._validate_sensor_context(rollback, health, previous=original_dependencies)

    def test_health_change_and_expiry_fail_closed_for_new_planning(self) -> None:
        degraded = self._health()
        degraded.health_version += 1
        degraded.health = self.sensing.SCOUT_SENSOR_HEALTH_DEGRADED
        degraded.health_content_identity.sha256 = canonical_business_identity(
            degraded, "health_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "not nominal"):
            self._validate_health(degraded)

        uncalibrated = self._health()
        uncalibrated.health = self.sensing.SCOUT_SENSOR_HEALTH_UNCALIBRATED
        uncalibrated.health_content_identity.sha256 = canonical_business_identity(
            uncalibrated, "health_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "not nominal"):
            self._validate_health(uncalibrated)

        with self.assertRaisesRegex(ValueError, "stale"):
            self._validate_health(self._health(), now=1_200_000_001)

    def test_current_requires_complete_error_bounds_and_optional_gradient(self) -> None:
        no_gradient = self._current()
        no_gradient.ClearField("spatial_gradient")
        no_gradient.current_content_identity.sha256 = canonical_business_identity(
            no_gradient, "current_content_identity"
        )
        self._validate_current(no_gradient)

        incomplete = self._current()
        incomplete.component_error_bound_mps.ClearField("z_mps")
        incomplete.current_content_identity.sha256 = canonical_business_identity(
            incomplete, "current_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "error bound"):
            self._validate_current(incomplete)

        invalid_gradient = self._current()
        invalid_gradient.spatial_gradient.row_major_per_s.pop()
        invalid_gradient.current_content_identity.sha256 = canonical_business_identity(
            invalid_gradient, "current_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "gradient"):
            self._validate_current(invalid_gradient)

    def test_current_expiry_domain_frame_and_validity_fail_closed(self) -> None:
        with self.assertRaisesRegex(ValueError, "stale|out-of-domain"):
            self._validate_current(self._current(), now=1_200_000_001)

        wrong_domain = self._current()
        wrong_domain.operating_domain_id = "sea-trial/unknown"
        wrong_domain.current_content_identity.sha256 = canonical_business_identity(
            wrong_domain, "current_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "out-of-domain"):
            self._validate_current(wrong_domain)

        wrong_frame = self._current()
        wrong_frame.velocity_at_reference_mps.frame_id = "ned"
        wrong_frame.current_content_identity.sha256 = canonical_business_identity(
            wrong_frame, "current_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "error bound"):
            self._validate_current(wrong_frame)

        degraded = self._current()
        degraded.validity = self.sensing.CURRENT_ESTIMATE_DEGRADED
        degraded.current_content_identity.sha256 = canonical_business_identity(
            degraded, "current_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "invalid"):
            self._validate_current(degraded)

    def test_non_production_profile_cannot_be_used_as_production_evidence(self) -> None:
        self.assertFalse(self.profile["production"])
        self.assertFalse(self._geometry().production_approved)
        for prefix in ("scout_sensor_health", "scout_current"):
            self.assertLess(
                self.profile["timing"][f"{prefix}_publish_period_ns"],
                self.profile["timing"][f"{prefix}_stale_warning_ns"],
            )
            self.assertLess(
                self.profile["timing"][f"{prefix}_stale_warning_ns"],
                self.profile["timing"][f"{prefix}_reject_ns"],
            )
        with self.assertRaisesRegex(ValueError, "NON_PRODUCTION"):
            self._validate_geometry(self._geometry(), production=True)
        with self.assertRaisesRegex(ValueError, "NON_PRODUCTION"):
            self._validate_current(self._current(), production=True)

        forged_profile = json.loads(json.dumps(self.profile))
        forged_profile["production"] = True
        forged_geometry = self._geometry()
        forged_geometry.production_approved = True
        forged_geometry.geometry_content_identity.sha256 = canonical_business_identity(
            forged_geometry, "geometry_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "Manifest-bound"):
            validate_sensor_geometry(
                forged_geometry,
                self.sensing,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=forged_profile,
                profile_artifact_bytes=self.profile_artifact_bytes,
                production_planning=True,
            )
        with self.assertRaisesRegex(ValueError, "Manifest-bound"):
            validate_current_estimate(
                self._current(),
                self.sensing,
                self.common,
                clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                profile=forged_profile,
                profile_artifact_bytes=self.profile_artifact_bytes,
                production_planning=True,
            )

    def test_identity_manifest_unknown_fields_and_resource_limits_fail_closed(self) -> None:
        tampered = self._geometry()
        tampered.field_of_view.maximum_range_m += 1.0
        with self.assertRaisesRegex(ValueError, "content identity"):
            self._validate_geometry(tampered)

        unknown = self.sensing.ScoutCurrentEstimate.FromString(
            self._current().SerializeToString(deterministic=True) + b"\xa0\x06\x01"
        )
        unknown.current_content_identity.sha256 = canonical_business_identity(
            unknown, "current_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "unknown field"):
            self._validate_current(unknown)

        oversized = self._geometry()
        oversized.sensor_id = "x" * (self.profile["interface_limits"]["maximum_string_bytes"] + 1)
        oversized.geometry_content_identity.sha256 = canonical_business_identity(
            oversized, "geometry_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "RESOURCE_LIMIT_EXCEEDED"):
            self._validate_geometry(oversized)

        oversized_health = self._health()
        oversized_health.sensor_id = "x" * (
            self.profile["interface_limits"]["maximum_string_bytes"] + 1
        )
        oversized_health.health_content_identity.sha256 = canonical_business_identity(
            oversized_health, "health_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "RESOURCE_LIMIT_EXCEEDED"):
            self._validate_health(oversized_health)

        unregistered_fault = self._health()
        unregistered_fault.active_fault_codes.append(4_294_967_295)
        unregistered_fault.health_content_identity.sha256 = canonical_business_identity(
            unregistered_fault, "health_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "active fault codes"):
            self._validate_health(unregistered_fault)

        wrong_manifest = self._current()
        wrong_manifest.header.manifest.manifest_identity.sha256 = b"x" * 32
        with self.assertRaisesRegex(ValueError, "manifest"):
            self._validate_current(wrong_manifest)

        self.assertIn(
            "scout_sensor_and_current_v1", self.manifest["supported_features"]
        )

    def test_normative_docs_define_context_revalidation_hashing_and_adapters(self) -> None:
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        for rule in (
            "geometry_version",
            "health_version",
            "Any change to either dependency requires plan revalidation",
            "Uncalibrated, stale, out-of-domain, frame-mismatched",
            "MUST NOT compare monotonic ticks across clock domains",
            "C++ <-> Protobuf",
            "ROS 2 <-> Protobuf",
            "Adapters MUST NOT synthesize missing values",
            "NON_PRODUCTION",
        ):
            self.assertIn(rule, contract)

        hashing = HASHING_PATH.read_text(encoding="utf-8")
        for identity_rule in (
            "`ScoutSensorGeometry`: clear `header` and `geometry_content_identity`",
            "`ScoutSensorHealthState`: clear `header` and `health_content_identity`",
            "`ScoutCurrentEstimate`: clear `header` and `current_content_identity`",
        ):
            self.assertIn(identity_rule, hashing)

    def test_stream_watermarks_reject_reorder_conflict_rollback_and_retired_session(self) -> None:
        for message, version_field, identity_field in (
            (self._geometry(), "geometry_version", "geometry_content_identity"),
            (self._health(), "health_version", "health_content_identity"),
            (self._current(), "current_model_version", "current_content_identity"),
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

                reordered = dict(arguments, incoming_sequence=message.header.sequence - 1)
                with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
                    apply_versioned_stream_watermark(**reordered)

                conflict = dict(arguments, incoming_delivery_identity=b"x" * 32)
                with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
                    apply_versioned_stream_watermark(**conflict)

                rollback = dict(
                    arguments,
                    incoming_sequence=message.header.sequence + 1,
                    incoming_version=version - 1,
                    incoming_identity=b"y" * 32,
                    incoming_delivery_identity=b"z" * 32,
                )
                with self.assertRaisesRegex(ValueError, "VERSION_INCOMPATIBLE"):
                    apply_versioned_stream_watermark(**rollback)

                retired = {b"fedcba9876543210"}
                replay = dict(
                    arguments,
                    incoming_session=b"fedcba9876543210",
                    incoming_sequence=1,
                    incoming_version=1,
                    incoming_identity=b"a" * 32,
                    incoming_delivery_identity=b"b" * 32,
                    retired_sessions=retired,
                )
                with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
                    apply_versioned_stream_watermark(**replay)

                new_session_rollback = dict(
                    arguments,
                    incoming_session=b"fedcba9876543210",
                    incoming_sequence=1,
                    incoming_version=version - 1,
                    incoming_identity=b"c" * 32,
                    incoming_delivery_identity=b"d" * 32,
                )
                with self.assertRaisesRegex(ValueError, "VERSION_INCOMPATIBLE"):
                    apply_versioned_stream_watermark(**new_session_rollback)

                new_session_same_business_content = dict(
                    arguments,
                    incoming_session=b"fedcba9876543210",
                    incoming_sequence=1,
                    incoming_delivery_identity=b"e" * 32,
                )
                self.assertEqual(
                    apply_versioned_stream_watermark(
                        **new_session_same_business_content
                    ),
                    "new session",
                )


if __name__ == "__main__":
    unittest.main()
