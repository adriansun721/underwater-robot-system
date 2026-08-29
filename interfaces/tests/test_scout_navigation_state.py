"""Public-contract tests for authoritative Scout navigation snapshots."""

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

from google.protobuf.descriptor import FieldDescriptor


INTERFACES = pathlib.Path(__file__).resolve().parents[1]
PROTO_ROOT = INTERFACES / "proto"
PROTO_V1 = PROTO_ROOT / "underwater" / "contracts" / "v1"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
CONTRACT_PATH = INTERFACES / "SCOUT_NAVIGATION_STATE.md"
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


def canonical_navigation_identity(snapshot: object) -> bytes:
    canonical = type(snapshot)()
    canonical.CopyFrom(snapshot)
    canonical.ClearField("header")
    canonical.ClearField("navigation_content_identity")
    _normalize_canonical_message(canonical)
    return hashlib.sha256(canonical.SerializeToString(deterministic=True)).digest()


def exact_delivery_identity(message: object) -> bytes:
    return hashlib.sha256(message.SerializeToString(deterministic=True)).digest()


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


def enu_to_ned(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    east, north, up = vector
    return north, east, -up


def ned_to_enu(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    north, east, down = vector
    return east, north, -down


def flu_to_frd(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    forward, left, up = vector
    return forward, -left, -up


def frd_to_flu(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    forward, right, down = vector
    return forward, -right, -down


def normalize_yaw(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def enu_yaw_to_ned(yaw_rad: float) -> float:
    return normalize_yaw(math.pi / 2.0 - yaw_rad)


def apply_navigation_watermark(
    current_session: bytes,
    current_version: int,
    current_identity: bytes,
    incoming_session: bytes,
    incoming_version: int,
    incoming_identity: bytes,
    *,
    current_message_sequence: int,
    incoming_message_sequence: int,
    current_delivery_identity: bytes,
    incoming_delivery_identity: bytes,
    retired_sessions: set[bytes],
) -> str:
    if (
        current_message_sequence == 0
        or incoming_message_sequence == 0
        or len(current_delivery_identity) != 32
        or len(incoming_delivery_identity) != 32
    ):
        raise ValueError("SEQUENCE_REJECTED: zero navigation message sequence")
    if incoming_session != current_session:
        if incoming_session in retired_sessions:
            raise ValueError("SEQUENCE_REJECTED: retired navigation session replay")
        retired_sessions.add(current_session)
        return "new session"
    if incoming_message_sequence < current_message_sequence:
        raise ValueError("SEQUENCE_REJECTED: navigation message reorder")
    if incoming_message_sequence == current_message_sequence:
        if incoming_version == current_version and hmac.compare_digest(
            incoming_identity, current_identity
        ) and hmac.compare_digest(
            incoming_delivery_identity, current_delivery_identity
        ):
            return "idempotent duplicate"
        raise ValueError("INPUT_INVALID: navigation sequence identity conflict")
    if incoming_version < current_version:
        raise ValueError("VERSION_INCOMPATIBLE: navigation version rollback")
    if incoming_version == current_version:
        raise ValueError("INPUT_INVALID: new delivery reused navigation version")
    return "accepted"


def _is_symmetric_positive_semidefinite_3x3(values: list[float]) -> bool:
    if len(values) != 9 or not all(math.isfinite(value) for value in values):
        return False
    scale = max(1.0, *(abs(value) for value in values))
    scalar_tolerance = 1e-12 * scale
    minor_tolerance = 1e-12 * scale * scale
    determinant_tolerance = 1e-12 * scale * scale * scale
    if any(abs(values[row * 3 + column] - values[column * 3 + row]) > scalar_tolerance for row in range(3) for column in range(3)):
        return False
    if any(values[index * 3 + index] < -scalar_tolerance for index in range(3)):
        return False
    for first, second in ((0, 1), (0, 2), (1, 2)):
        minor = values[first * 3 + first] * values[second * 3 + second] - values[first * 3 + second] ** 2
        if minor < -minor_tolerance:
            return False
    determinant = (
        values[0] * (values[4] * values[8] - values[5] * values[7])
        - values[1] * (values[3] * values[8] - values[5] * values[6])
        + values[2] * (values[3] * values[7] - values[4] * values[6])
    )
    return determinant >= -determinant_tolerance


def validate_navigation_snapshot(
    snapshot: object,
    state: object,
    common: object,
    *,
    consumer_clock_domain_id: str,
    now_monotonic_ns: int,
    contract_manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    timing_profile: dict[str, object],
) -> None:
    """Executable planning-consumer gate for public conformance vectors."""
    if _has_unknown_fields(snapshot):
        raise ValueError("unknown field in navigation snapshot")
    limits = timing_profile["interface_limits"]
    if (
        len(snapshot.SerializeToString(deterministic=True))
        > limits["maximum_navigation_state_bytes"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(snapshot)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: navigation snapshot")
    header = snapshot.header
    manifest_profile = contract_manifest["integration_profile"]
    reject_age_ns = timing_profile["timing"]["scout_navigation_reject_ns"]
    if (
        header.schema_major != contract_manifest["schema_major"]
        or header.schema_minor != contract_manifest["schema_minor"]
        or header.stream_id != common.STREAM_SCOUT_NAVIGATION_STATE
        or not header.producer_id
        or len(header.producer_session_id) != 16
        or header.source_clock_domain_id != consumer_clock_domain_id
        or not header.HasField("manifest")
        or header.manifest.schema_major != contract_manifest["schema_major"]
        or header.manifest.schema_minor != contract_manifest["schema_minor"]
        or not hmac.compare_digest(
            header.manifest.manifest_identity.sha256,
            accepted_manifest_identity,
        )
        or "scout_navigation_state_v1" not in contract_manifest["supported_features"]
        or not header.HasField("generated_at_monotonic_ns")
        or not header.HasField("observed_at")
        or not header.observed_at.HasField("utc_time_ns")
        or not header.observed_at.HasField("uncertainty_ns")
        or header.observed_at.status
        not in {
            common.TIME_SYNC_UNSYNCHRONIZED,
            common.TIME_SYNC_SYNCHRONIZED,
            common.TIME_SYNC_DEGRADED,
        }
    ):
        raise ValueError("invalid navigation source identity, clock, or manifest")
    if (
        snapshot.navigation_version == 0
        or not snapshot.HasField("observed_at_monotonic_ns")
        or snapshot.observed_at_monotonic_ns < 0
        or header.generated_at_monotonic_ns < snapshot.observed_at_monotonic_ns
        or now_monotonic_ns < snapshot.observed_at_monotonic_ns
        or now_monotonic_ns - snapshot.observed_at_monotonic_ns > reject_age_ns
    ):
        raise ValueError("stale or invalid navigation observation time")
    if (
        snapshot.timing_profile.profile_id != timing_profile["profile_id"]
        or snapshot.timing_profile.profile_id != manifest_profile["id"]
        or snapshot.timing_profile.version != timing_profile["version"]
        or snapshot.timing_profile.version != manifest_profile["version"]
        or not hmac.compare_digest(
            snapshot.timing_profile.content_identity.sha256,
            bytes.fromhex(manifest_profile["sha256"]),
        )
    ):
        raise ValueError("invalid navigation profile")

    pose_fields = ("x_m", "y_m", "z_m", "q_x", "q_y", "q_z", "q_w")
    twist_fields = (
        "linear_x_mps",
        "linear_y_mps",
        "linear_z_mps",
        "angular_x_radps",
        "angular_y_radps",
        "angular_z_radps",
    )
    if (
        snapshot.pose.world_frame_id != "mission_enu"
        or snapshot.pose.body_frame_id != "base_link"
        or snapshot.body_twist.frame_id != "base_link"
        or snapshot.position_covariance_m2.frame_id != "mission_enu"
        or snapshot.attitude_covariance_rad2.frame_id != "base_link"
        or not all(snapshot.pose.HasField(field) and math.isfinite(getattr(snapshot.pose, field)) for field in pose_fields)
        or not all(snapshot.body_twist.HasField(field) and math.isfinite(getattr(snapshot.body_twist, field)) for field in twist_fields)
    ):
        raise ValueError("invalid navigation frame or non-finite kinematics")
    quaternion_norm_squared = sum(
        getattr(snapshot.pose, field) ** 2 for field in ("q_x", "q_y", "q_z", "q_w")
    )
    if abs(quaternion_norm_squared - 1.0) > 1e-6:
        raise ValueError("invalid navigation attitude")
    if not _is_symmetric_positive_semidefinite_3x3(list(snapshot.position_covariance_m2.row_major)):
        raise ValueError("invalid position covariance")
    if not _is_symmetric_positive_semidefinite_3x3(list(snapshot.attitude_covariance_rad2.row_major)):
        raise ValueError("invalid attitude covariance")
    if snapshot.validity != state.NAVIGATION_SOLUTION_VALID:
        raise ValueError("navigation solution is not valid for planning")
    if (
        len(snapshot.navigation_content_identity.sha256) != 32
        or not hmac.compare_digest(
            snapshot.navigation_content_identity.sha256,
            canonical_navigation_identity(snapshot),
        )
    ):
        raise ValueError("invalid navigation content identity")


class ScoutNavigationStateContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.generated = tempfile.TemporaryDirectory()
        proto_files = sorted(str(path) for path in PROTO_V1.glob("*.proto"))
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
        cls.state = importlib.import_module("underwater.contracts.v1.state_pb2")
        cls.profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        cls.accepted_manifest_identity = b"m" * 32

    @classmethod
    def tearDownClass(cls) -> None:
        sys.path.remove(cls.generated.name)
        cls.generated.cleanup()

    def _complete_snapshot(self) -> object:
        snapshot = self.state.ScoutNavigationState(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-localization-authority",
                producer_session_id=b"0123456789abcdef",
                stream_id=self.common.STREAM_SCOUT_NAVIGATION_STATE,
                sequence=81,
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
            ),
            navigation_version=37,
            observed_at_monotonic_ns=1_000_000_000,
            pose=self.state.Pose3dEnuFlu(
                x_m=4.0,
                y_m=-2.0,
                z_m=-6.5,
                q_x=0.0,
                q_y=0.0,
                q_z=0.3826834323650898,
                q_w=0.9238795325112867,
                world_frame_id="mission_enu",
                body_frame_id="base_link",
            ),
            body_twist=self.state.BodyTwist3dFlu(
                linear_x_mps=0.5,
                linear_y_mps=-0.1,
                linear_z_mps=0.02,
                angular_x_radps=0.01,
                angular_y_radps=-0.02,
                angular_z_radps=0.03,
                frame_id="base_link",
            ),
            position_covariance_m2=self.state.Covariance3d(
                row_major=[0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.09],
                frame_id="mission_enu",
            ),
            attitude_covariance_rad2=self.state.Covariance3d(
                row_major=[0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02],
                frame_id="base_link",
            ),
            validity=self.state.NAVIGATION_SOLUTION_VALID,
            timing_profile=self.common.ProfileRef(
                profile_id=self.profile["profile_id"],
                version=self.profile["version"],
                content_identity=self.common.ContentIdentity(
                    sha256=bytes.fromhex(
                        self.manifest["integration_profile"]["sha256"]
                    )
                ),
            ),
        )
        snapshot.navigation_content_identity.sha256 = canonical_navigation_identity(snapshot)
        return snapshot

    def test_complete_navigation_snapshot_round_trip_preserves_public_state(self) -> None:
        snapshot = self._complete_snapshot()

        decoded = self.state.ScoutNavigationState.FromString(
            snapshot.SerializeToString(deterministic=True)
        )

        self.assertEqual(decoded, snapshot)
        self.assertEqual(decoded.pose.world_frame_id, "mission_enu")
        self.assertEqual(decoded.body_twist.frame_id, "base_link")
        self.assertEqual(len(decoded.position_covariance_m2.row_major), 9)
        self.assertEqual(decoded.header.source_clock_domain_id, "scout-nuc/boot-12")
        self.assertEqual(decoded.navigation_content_identity.sha256, snapshot.navigation_content_identity.sha256)

    def test_fresh_valid_snapshot_is_accepted_under_bound_timing_profile(self) -> None:
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        timing = profile["timing"]

        self.assertLess(
            timing["scout_navigation_publish_period_ns"],
            timing["scout_navigation_stale_warning_ns"],
        )
        self.assertLess(
            timing["scout_navigation_stale_warning_ns"],
            timing["scout_navigation_reject_ns"],
        )
        validate_navigation_snapshot(
            self._complete_snapshot(),
            self.state,
            self.common,
            consumer_clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=1_030_000_000,
            contract_manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            timing_profile=self.profile,
        )
        boundary = self._complete_snapshot()
        validate_navigation_snapshot(
            boundary,
            self.state,
            self.common,
            consumer_clock_domain_id="scout-nuc/boot-12",
            now_monotonic_ns=(
                boundary.observed_at_monotonic_ns
                + timing["scout_navigation_reject_ns"]
            ),
            contract_manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            timing_profile=self.profile,
        )

    def test_invalid_state_is_rejected_as_a_whole_without_defaults(self) -> None:
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        reject_age_ns = profile["timing"]["scout_navigation_reject_ns"]

        def assert_rejected(snapshot: object, pattern: str, **overrides: object) -> None:
            snapshot.navigation_content_identity.sha256 = canonical_navigation_identity(snapshot)
            arguments = {
                "consumer_clock_domain_id": "scout-nuc/boot-12",
                "now_monotonic_ns": 1_030_000_000,
                "contract_manifest": self.manifest,
                "accepted_manifest_identity": self.accepted_manifest_identity,
                "timing_profile": self.profile,
            }
            arguments.update(overrides)
            with self.assertRaisesRegex(ValueError, pattern):
                validate_navigation_snapshot(
                    snapshot,
                    self.state,
                    self.common,
                    **arguments,
                )

        wrong_world = self._complete_snapshot()
        wrong_world.pose.world_frame_id = "map"
        assert_rejected(wrong_world, "frame")

        wrong_body = self._complete_snapshot()
        wrong_body.body_twist.frame_id = "base_link_frd"
        assert_rejected(wrong_body, "frame")

        non_finite = self._complete_snapshot()
        non_finite.pose.x_m = math.nan
        assert_rejected(non_finite, "non-finite")

        invalid_attitude = self._complete_snapshot()
        invalid_attitude.pose.q_w = 0.0
        assert_rejected(invalid_attitude, "attitude")

        asymmetric = self._complete_snapshot()
        asymmetric.position_covariance_m2.row_major[1] = 0.01
        assert_rejected(asymmetric, "position covariance")

        large_scale_asymmetric = self._complete_snapshot()
        large_scale_asymmetric.position_covariance_m2.row_major[:] = [
            1e9,
            1e6,
            0.0,
            0.0,
            1e9,
            0.0,
            0.0,
            0.0,
            1e9,
        ]
        assert_rejected(large_scale_asymmetric, "position covariance")

        non_psd = self._complete_snapshot()
        non_psd.attitude_covariance_rad2.row_major[8] = -0.02
        assert_rejected(non_psd, "attitude covariance")

        degraded = self._complete_snapshot()
        degraded.validity = self.state.NAVIGATION_SOLUTION_DEGRADED
        assert_rejected(degraded, "not valid for planning")

        unknown_validity = self._complete_snapshot()
        unknown_validity.validity = 99
        assert_rejected(unknown_validity, "not valid for planning")

        wrong_clock = self._complete_snapshot()
        assert_rejected(
            wrong_clock,
            "clock",
            consumer_clock_domain_id="scout-nuc/boot-13",
        )

        stale = self._complete_snapshot()
        assert_rejected(
            stale,
            "stale",
            now_monotonic_ns=stale.observed_at_monotonic_ns + reject_age_ns + 1,
        )

        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        for rule in (
            "mission_enu",
            "base_link",
            "symmetric positive semidefinite",
            "The entire snapshot MUST be rejected",
            "MUST NOT compare monotonic ticks across clock domains",
            "NAVIGATION_SOLUTION_DEGRADED is not execution-authorizable",
        ):
            self.assertIn(rule, contract)

    def test_hash_identity_detects_business_tampering_and_manifest_gates_feature(self) -> None:
        snapshot = self._complete_snapshot()
        changed_header = self.state.ScoutNavigationState()
        changed_header.CopyFrom(snapshot)
        changed_header.header.sequence += 1
        self.assertEqual(
            canonical_navigation_identity(changed_header),
            snapshot.navigation_content_identity.sha256,
        )

        tampered = self.state.ScoutNavigationState()
        tampered.CopyFrom(snapshot)
        tampered.pose.z_m += 0.1
        with self.assertRaisesRegex(ValueError, "content identity"):
            validate_navigation_snapshot(
                tampered,
                self.state,
                self.common,
                consumer_clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                contract_manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                timing_profile=self.profile,
            )

        hashing = HASHING_PATH.read_text(encoding="utf-8")
        self.assertIn(
            "`ScoutNavigationState`: clear `header` and `navigation_content_identity`",
            hashing,
        )
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertIn("scout_navigation_state_v1", manifest["supported_features"])
        self.assertEqual(manifest["approved_mixed_versions"], [])

    def test_canonical_identity_normalizes_negative_zero(self) -> None:
        positive_zero = self._complete_snapshot()
        positive_zero.pose.x_m = 0.0
        negative_zero = self.state.ScoutNavigationState()
        negative_zero.CopyFrom(positive_zero)
        negative_zero.pose.x_m = -0.0

        self.assertEqual(
            canonical_navigation_identity(positive_zero),
            canonical_navigation_identity(negative_zero),
        )

        composed = self._complete_snapshot()
        composed.timing_profile.profile_id = "caf\u00e9"
        decomposed = self.state.ScoutNavigationState()
        decomposed.CopyFrom(composed)
        decomposed.timing_profile.profile_id = "cafe\u0301"
        self.assertEqual(
            canonical_navigation_identity(composed),
            canonical_navigation_identity(decomposed),
        )

    def test_unknown_wire_field_is_rejected_before_identity_installation(self) -> None:
        snapshot = self._complete_snapshot()
        # Unknown field 100, varint value 1.
        with_unknown = self.state.ScoutNavigationState.FromString(
            snapshot.SerializeToString(deterministic=True) + b"\xa0\x06\x01"
        )
        with_unknown.navigation_content_identity.sha256 = canonical_navigation_identity(
            with_unknown
        )

        with self.assertRaisesRegex(ValueError, "unknown field"):
            validate_navigation_snapshot(
                with_unknown,
                self.state,
                self.common,
                consumer_clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                contract_manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                timing_profile=self.profile,
            )

    def test_interface_limits_reject_oversized_navigation_snapshots(self) -> None:
        limits = self.profile["interface_limits"]
        self.assertGreater(limits["maximum_navigation_state_bytes"], 0)

        oversized = self._complete_snapshot()
        oversized.header.producer_id = "x" * (limits["maximum_string_bytes"] + 1)
        with self.assertRaisesRegex(ValueError, "RESOURCE_LIMIT_EXCEEDED"):
            validate_navigation_snapshot(
                oversized,
                self.state,
                self.common,
                consumer_clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                contract_manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                timing_profile=self.profile,
            )

    def test_adapter_requirements_are_bidirectional_and_field_complete(self) -> None:
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        for requirement in (
            "C++ <-> Protobuf",
            "ROS 2 <-> Protobuf",
            "Every field is mapped in both directions",
            "Adapters MUST NOT synthesize missing values",
            "adapter output MUST pass the same planning-consumer gate",
        ):
            self.assertIn(requirement, contract)

    def test_fcu_adapter_single_axis_and_direction_golden_vectors(self) -> None:
        enu_vectors = {
            (1.0, 0.0, 0.0): (0.0, 1.0, -0.0),
            (0.0, 1.0, 0.0): (1.0, 0.0, -0.0),
            (0.0, 0.0, 1.0): (0.0, 0.0, -1.0),
        }
        for source, expected in enu_vectors.items():
            with self.subTest(enu=source):
                self.assertEqual(enu_to_ned(source), expected)
                self.assertEqual(ned_to_enu(expected), source)

        flu_vectors = {
            (1.0, 0.0, 0.0): (1.0, -0.0, -0.0),
            (0.0, 1.0, 0.0): (0.0, -1.0, -0.0),
            (0.0, 0.0, 1.0): (0.0, -0.0, -1.0),
        }
        for source, expected in flu_vectors.items():
            with self.subTest(flu=source):
                self.assertEqual(flu_to_frd(source), expected)
                self.assertEqual(frd_to_flu(expected), source)

        self.assertAlmostEqual(enu_yaw_to_ned(0.0), math.pi / 2.0)
        self.assertAlmostEqual(enu_yaw_to_ned(math.pi / 2.0), 0.0)
        self.assertAlmostEqual(enu_yaw_to_ned(-math.pi / 2.0), -math.pi)
        self.assertEqual(flu_to_frd((0.0, 0.0, 1.0))[2], -1.0)

        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        for golden_rule in (
            "(E, N, U) -> (N, E, -U)",
            "(F, L, U) -> (F, -L, -U)",
            "yaw_ned = pi/2 - yaw_enu",
            "Every FCU adapter MUST run these vectors in both directions",
        ):
            self.assertIn(golden_rule, contract)

    def test_missing_contract_manifest_reference_is_rejected(self) -> None:
        snapshot = self._complete_snapshot()
        snapshot.header.ClearField("manifest")
        snapshot.navigation_content_identity.sha256 = canonical_navigation_identity(snapshot)

        with self.assertRaisesRegex(ValueError, "manifest"):
            validate_navigation_snapshot(
                snapshot,
                self.state,
                self.common,
                consumer_clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                contract_manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                timing_profile=self.profile,
            )

    def test_wrong_exact_manifest_identity_is_rejected(self) -> None:
        snapshot = self._complete_snapshot()
        snapshot.header.manifest.manifest_identity.sha256 = b"x" * 32

        with self.assertRaisesRegex(ValueError, "manifest"):
            validate_navigation_snapshot(
                snapshot,
                self.state,
                self.common,
                consumer_clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                contract_manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                timing_profile=self.profile,
            )

        wrong_profile = self._complete_snapshot()
        wrong_profile.timing_profile.content_identity.sha256 = b"p" * 32
        wrong_profile.navigation_content_identity.sha256 = canonical_navigation_identity(
            wrong_profile
        )
        with self.assertRaisesRegex(ValueError, "profile"):
            validate_navigation_snapshot(
                wrong_profile,
                self.state,
                self.common,
                consumer_clock_domain_id="scout-nuc/boot-12",
                now_monotonic_ns=1_030_000_000,
                contract_manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                timing_profile=self.profile,
            )

    def test_navigation_version_watermark_is_session_scoped_and_fail_closed(self) -> None:
        session = b"0123456789abcdef"
        identity = b"a" * 32
        retired_sessions: set[bytes] = set()

        self.assertEqual(
            apply_navigation_watermark(
                session,
                7,
                identity,
                session,
                8,
                b"b" * 32,
                current_message_sequence=10,
                incoming_message_sequence=11,
                current_delivery_identity=b"d" * 32,
                incoming_delivery_identity=b"e" * 32,
                retired_sessions=retired_sessions,
            ),
            "accepted",
        )
        self.assertEqual(
            apply_navigation_watermark(
                session,
                7,
                identity,
                session,
                7,
                identity,
                current_message_sequence=10,
                incoming_message_sequence=10,
                current_delivery_identity=b"d" * 32,
                incoming_delivery_identity=b"d" * 32,
                retired_sessions=retired_sessions,
            ),
            "idempotent duplicate",
        )
        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            apply_navigation_watermark(
                session,
                7,
                identity,
                session,
                7,
                identity,
                current_message_sequence=10,
                incoming_message_sequence=10,
                current_delivery_identity=b"d" * 32,
                incoming_delivery_identity=b"e" * 32,
                retired_sessions=retired_sessions,
            )
        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            apply_navigation_watermark(
                session,
                7,
                identity,
                session,
                7,
                b"x" * 32,
                current_message_sequence=10,
                incoming_message_sequence=10,
                current_delivery_identity=b"d" * 32,
                incoming_delivery_identity=b"e" * 32,
                retired_sessions=retired_sessions,
            )
        with self.assertRaisesRegex(ValueError, "VERSION_INCOMPATIBLE"):
            apply_navigation_watermark(
                session,
                7,
                identity,
                session,
                6,
                b"z" * 32,
                current_message_sequence=10,
                incoming_message_sequence=11,
                current_delivery_identity=b"d" * 32,
                incoming_delivery_identity=b"e" * 32,
                retired_sessions=retired_sessions,
            )
        self.assertEqual(
            apply_navigation_watermark(
                session,
                7,
                identity,
                b"fedcba9876543210",
                1,
                b"c" * 32,
                current_message_sequence=10,
                incoming_message_sequence=1,
                current_delivery_identity=b"d" * 32,
                incoming_delivery_identity=b"e" * 32,
                retired_sessions=retired_sessions,
            ),
            "new session",
        )
        self.assertIn(session, retired_sessions)
        with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
            apply_navigation_watermark(
                b"fedcba9876543210",
                1,
                b"c" * 32,
                session,
                8,
                b"b" * 32,
                current_message_sequence=1,
                incoming_message_sequence=12,
                current_delivery_identity=b"d" * 32,
                incoming_delivery_identity=b"e" * 32,
                retired_sessions=retired_sessions,
            )
        with self.assertRaisesRegex(ValueError, "SEQUENCE_REJECTED"):
            apply_navigation_watermark(
                session,
                7,
                identity,
                session,
                8,
                b"b" * 32,
                current_message_sequence=10,
                incoming_message_sequence=9,
                current_delivery_identity=b"d" * 32,
                incoming_delivery_identity=b"e" * 32,
                retired_sessions=set(),
            )

    def test_same_sequence_header_tampering_is_not_an_idempotent_retransmission(self) -> None:
        original = self._complete_snapshot()
        tampered_header = self.state.ScoutNavigationState()
        tampered_header.CopyFrom(original)
        tampered_header.header.generated_at_monotonic_ns += 1

        self.assertEqual(
            original.navigation_content_identity,
            tampered_header.navigation_content_identity,
        )
        self.assertNotEqual(
            exact_delivery_identity(original),
            exact_delivery_identity(tampered_header),
        )
        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            apply_navigation_watermark(
                original.header.producer_session_id,
                original.navigation_version,
                original.navigation_content_identity.sha256,
                tampered_header.header.producer_session_id,
                tampered_header.navigation_version,
                tampered_header.navigation_content_identity.sha256,
                current_message_sequence=original.header.sequence,
                incoming_message_sequence=tampered_header.header.sequence,
                current_delivery_identity=exact_delivery_identity(original),
                incoming_delivery_identity=exact_delivery_identity(tampered_header),
                retired_sessions=set(),
            )


if __name__ == "__main__":
    unittest.main()
