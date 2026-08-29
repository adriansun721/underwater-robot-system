"""Public-contract tests for main-robot prediction and Scout coordination."""

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
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"
CONTRACT_PATH = INTERFACES / "SCOUT_MAIN_ROBOT_COORDINATION.md"
HASHING_PATH = INTERFACES / "HASHING.md"


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


def _has_unknown_fields(message: object) -> bool:
    known_only = type(message)()
    known_only.CopyFrom(message)
    known_only.DiscardUnknownFields()
    return known_only.SerializeToString(deterministic=True) != message.SerializeToString(
        deterministic=True
    )


def _message_strings(message: object) -> list[str]:
    values: list[str] = []
    for field, value in message.ListFields():
        if field.label == FieldDescriptor.LABEL_REPEATED:
            if field.type == FieldDescriptor.TYPE_MESSAGE:
                for item in value:
                    values.extend(_message_strings(item))
            elif field.type == FieldDescriptor.TYPE_STRING:
                values.extend(value)
        elif field.type == FieldDescriptor.TYPE_MESSAGE:
            values.extend(_message_strings(value))
        elif field.type == FieldDescriptor.TYPE_STRING:
            values.append(value)
    return values


def _profile_ref_is_valid(reference: object) -> bool:
    return (
        bool(reference.profile_id)
        and reference.version > 0
        and len(reference.content_identity.sha256) == 32
    )


def _header_is_valid(
    header: object,
    *,
    stream_id: int,
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
        and bool(header.source_clock_domain_id)
        and header.HasField("generated_at_monotonic_ns")
        and header.HasField("manifest")
        and header.manifest.schema_major == manifest["schema_major"]
        and header.manifest.schema_minor == manifest["schema_minor"]
        and hmac.compare_digest(
            header.manifest.manifest_identity.sha256,
            accepted_manifest_identity,
        )
        and "scout_main_robot_coordination_v1" in manifest["supported_features"]
    )


def _validate_local_age(
    *, received_at_local_ns: int, now_local_ns: int, reject_age_ns: int
) -> None:
    age_ns = now_local_ns - received_at_local_ns
    if age_ns < 0:
        raise ValueError("CLOCK_DOMAIN_MISMATCH: local receive age is negative")
    if age_ns > reject_age_ns:
        raise ValueError("DEPENDENCY_STALE: local receive age exceeded")


def _point_tuple(point: object) -> tuple[float, float, float]:
    if (
        point.frame_id != "mission_enu"
        or not all(point.HasField(field) for field in ("x_m", "y_m", "z_m"))
        or not all(
            math.isfinite(getattr(point, field)) for field in ("x_m", "y_m", "z_m")
        )
    ):
        raise ValueError("INPUT_INVALID: occupied point")
    return point.x_m, point.y_m, point.z_m


def validate_main_robot_prediction(
    prediction: object,
    common: object,
    *,
    mission_identity: tuple[int, int, bytes],
    received_at_local_ns: int,
    now_local_ns: int,
    profile: dict[str, object],
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
) -> None:
    limits = profile["interface_limits"]
    timing = profile["timing"]
    if _has_unknown_fields(prediction):
        raise ValueError("INPUT_INVALID: unknown prediction field")
    if any(value != unicodedata.normalize("NFC", value) for value in _message_strings(prediction)):
        raise ValueError("INPUT_INVALID: non-NFC prediction string")
    if (
        len(prediction.SerializeToString(deterministic=True))
        > limits["maximum_main_robot_prediction_bytes"]
        or len(prediction.occupied_intervals) > limits["maximum_prediction_intervals"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(prediction)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: prediction")
    if not _header_is_valid(
        prediction.header,
        stream_id=common.STREAM_MAIN_ROBOT_PREDICTION,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("INPUT_INVALID: prediction header or Manifest")
    _validate_local_age(
        received_at_local_ns=received_at_local_ns,
        now_local_ns=now_local_ns,
        reject_age_ns=timing["main_robot_prediction_reject_ns"],
    )
    if (
        (prediction.mission_id, prediction.mission_version, prediction.mission_content_identity.sha256)
        != mission_identity
        or not prediction.prediction_id
        or not prediction.main_robot_id
        or prediction.prediction_version == 0
        or len(prediction.prediction_content_identity.sha256) != 32
    ):
        raise ValueError("INPUT_INVALID: mission or prediction identity")
    if (
        not prediction.HasField("alignment_epoch")
        or not prediction.alignment_epoch.HasField("utc_time_ns")
        or not prediction.alignment_epoch.HasField("uncertainty_ns")
        or prediction.alignment_epoch.status != common.TIME_SYNC_SYNCHRONIZED
        or prediction.alignment_epoch.uncertainty_ns
        > timing["coordination_maximum_sync_uncertainty_ns"]
    ):
        raise ValueError("CLOCK_DOMAIN_MISMATCH: prediction alignment epoch")
    if (
        not prediction.HasField("source_valid_from_monotonic_ns")
        or not prediction.HasField("source_valid_until_monotonic_ns")
        or prediction.source_valid_from_monotonic_ns
        > prediction.header.generated_at_monotonic_ns
        or prediction.header.generated_at_monotonic_ns
        >= prediction.source_valid_until_monotonic_ns
    ):
        raise ValueError("INPUT_INVALID: prediction source validity")
    intervals = prediction.occupied_intervals
    if not intervals:
        raise ValueError("INPUT_INVALID: empty prediction")
    previous_end = 0
    previous_center: tuple[float, float, float] | None = None
    for index, interval in enumerate(intervals):
        start = _point_tuple(interval.swept_volume.start_center)
        end = _point_tuple(interval.swept_volume.end_center)
        volume = interval.swept_volume
        radii = (
            "physical_radius_m",
            "position_uncertainty_radius_m",
            "conservative_occupied_radius_m",
        )
        if (
            interval.start_offset_ns != previous_end
            or interval.end_offset_ns <= interval.start_offset_ns
            or (index > 0 and start != previous_center)
            or not all(volume.HasField(field) for field in radii)
            or not all(math.isfinite(getattr(volume, field)) for field in radii)
            or volume.physical_radius_m <= 0.0
            or volume.position_uncertainty_radius_m < 0.0
            or volume.conservative_occupied_radius_m
            < volume.physical_radius_m + volume.position_uncertainty_radius_m
        ):
            raise ValueError("INPUT_INVALID: prediction gap or occupied volume")
        previous_end = interval.end_offset_ns
        previous_center = end
    if previous_end > (
        prediction.source_valid_until_monotonic_ns
        - prediction.source_valid_from_monotonic_ns
    ):
        raise ValueError("INPUT_INVALID: prediction exceeds source validity")
    if not hmac.compare_digest(
        prediction.prediction_content_identity.sha256,
        canonical_business_identity(prediction, "prediction_content_identity"),
    ):
        raise ValueError("INPUT_INVALID: prediction content identity")


def validate_scout_coordination_constraint(
    constraint: object,
    prediction: object,
    common: object,
    cooperation: object,
    state: object,
    *,
    mission_identity: tuple[int, int, bytes],
    expected_coordination_version: int,
    received_at_local_ns: int,
    now_local_ns: int,
    profile: dict[str, object],
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
) -> None:
    limits = profile["interface_limits"]
    if _has_unknown_fields(constraint):
        raise ValueError("INPUT_INVALID: unknown coordination field")
    if (
        len(constraint.SerializeToString(deterministic=True))
        > limits["maximum_coordination_constraint_bytes"]
        or any(
            len(value.encode("utf-8")) > limits["maximum_string_bytes"]
            for value in _message_strings(constraint)
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: coordination")
    if not _header_is_valid(
        constraint.header,
        stream_id=common.STREAM_SCOUT_COORDINATION_CONSTRAINT,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    ):
        raise ValueError("INPUT_INVALID: coordination header or Manifest")
    _validate_local_age(
        received_at_local_ns=received_at_local_ns,
        now_local_ns=now_local_ns,
        reject_age_ns=profile["timing"]["coordination_constraint_reject_ns"],
    )
    if constraint.header.source_clock_domain_id != prediction.header.source_clock_domain_id:
        raise ValueError("CLOCK_DOMAIN_MISMATCH: prediction and coordination")
    if (
        (constraint.mission_id, constraint.mission_version, constraint.mission_content_identity.sha256)
        != mission_identity
        or constraint.coordination_version != expected_coordination_version
        or constraint.prediction_id != prediction.prediction_id
        or constraint.prediction_version != prediction.prediction_version
        or not hmac.compare_digest(
            constraint.prediction_content_identity.sha256,
            prediction.prediction_content_identity.sha256,
        )
    ):
        raise ValueError("INPUT_INVALID: task or prediction pairing")
    if (
        constraint.channel_id != state.CHANNEL_MAIN_SCOUT_COOP
        or not constraint.HasField("minimum_separation_m")
        or not constraint.HasField("maximum_communication_distance_m")
        or not math.isfinite(constraint.minimum_separation_m)
        or not math.isfinite(constraint.maximum_communication_distance_m)
        or constraint.minimum_separation_m <= 0.0
        or constraint.maximum_communication_distance_m
        <= constraint.minimum_separation_m
        or constraint.link_assurance_basis
        not in {
            cooperation.LINK_ASSURANCE_GEOMETRIC_DISTANCE_ONLY,
            cooperation.LINK_ASSURANCE_CALIBRATED_LINK_MODEL,
        }
        or (
            constraint.link_assurance_basis
            == cooperation.LINK_ASSURANCE_GEOMETRIC_DISTANCE_ONLY
            and constraint.HasField("calibrated_link_model")
        )
        or (
            constraint.link_assurance_basis
            == cooperation.LINK_ASSURANCE_CALIBRATED_LINK_MODEL
            and not _profile_ref_is_valid(constraint.calibrated_link_model)
        )
        or not _profile_ref_is_valid(constraint.scout_loss_policy)
        or not _profile_ref_is_valid(constraint.main_loss_policy)
    ):
        raise ValueError("INPUT_INVALID: coordination limits or policy")
    if (
        not constraint.HasField("source_valid_from_monotonic_ns")
        or not constraint.HasField("source_valid_until_monotonic_ns")
        or constraint.source_valid_from_monotonic_ns
        > constraint.header.generated_at_monotonic_ns
        or constraint.header.generated_at_monotonic_ns
        >= constraint.source_valid_until_monotonic_ns
    ):
        raise ValueError("INPUT_INVALID: coordination source validity")
    if not hmac.compare_digest(
        constraint.coordination_content_identity.sha256,
        canonical_business_identity(constraint, "coordination_content_identity"),
    ):
        raise ValueError("INPUT_INVALID: coordination content identity")


def validate_coordination_context(
    prediction: object,
    constraint: object,
    common: object,
    cooperation: object,
    state: object,
    *,
    mission_identity: tuple[int, int, bytes],
    expected_coordination_version: int,
    received_prediction_at_local_ns: int,
    received_coordination_at_local_ns: int,
    now_local_ns: int,
    plan_end_offset_ns: int,
    profile: dict[str, object],
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
) -> None:
    validate_main_robot_prediction(
        prediction,
        common,
        mission_identity=mission_identity,
        received_at_local_ns=received_prediction_at_local_ns,
        now_local_ns=now_local_ns,
        profile=profile,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    )
    validate_scout_coordination_constraint(
        constraint,
        prediction,
        common,
        cooperation,
        state,
        mission_identity=mission_identity,
        expected_coordination_version=expected_coordination_version,
        received_at_local_ns=received_coordination_at_local_ns,
        now_local_ns=now_local_ns,
        profile=profile,
        manifest=manifest,
        accepted_manifest_identity=accepted_manifest_identity,
    )
    if plan_end_offset_ns < 0 or plan_end_offset_ns > prediction.occupied_intervals[-1].end_offset_ns:
        raise ValueError("DEPENDENCY_STALE: plan exceeds prediction horizon")


def occupied_sphere_at_offset(
    prediction: object, offset_ns: int
) -> tuple[tuple[float, float, float], float]:
    candidates: list[tuple[tuple[float, float, float], float]] = []
    for interval in prediction.occupied_intervals:
        if interval.start_offset_ns <= offset_ns <= interval.end_offset_ns:
            duration_ns = interval.end_offset_ns - interval.start_offset_ns
            fraction = (offset_ns - interval.start_offset_ns) / duration_ns
            start = _point_tuple(interval.swept_volume.start_center)
            end = _point_tuple(interval.swept_volume.end_center)
            center = tuple(
                start[index] + fraction * (end[index] - start[index])
                for index in range(3)
            )
            candidates.append(
                (center, interval.swept_volume.conservative_occupied_radius_m)
            )
    if not candidates:
        raise ValueError("DEPENDENCY_STALE: offset outside prediction horizon")
    return max(candidates, key=lambda candidate: candidate[1])


def coordination_allows_position(
    prediction: object,
    constraint: object,
    *,
    offset_ns: int,
    scout_position: tuple[float, float, float],
) -> bool:
    if len(scout_position) != 3 or not all(math.isfinite(value) for value in scout_position):
        raise ValueError("INPUT_INVALID: Scout position")
    center, occupied_radius_m = occupied_sphere_at_offset(prediction, offset_ns)
    center_distance_m = math.dist(center, scout_position)
    tolerance = 1e-12
    return (
        center_distance_m - occupied_radius_m + tolerance
        >= constraint.minimum_separation_m
        and center_distance_m
        <= constraint.maximum_communication_distance_m + tolerance
    )


def apply_stream_watermark(
    *,
    current_session: bytes,
    current_sequence: int,
    current_version: int,
    current_identity: bytes,
    incoming_session: bytes,
    incoming_sequence: int,
    incoming_version: int,
    incoming_identity: bytes,
    retired_sessions: set[bytes],
    recovery_boundary: bool,
) -> str:
    if (
        len(current_session) != 16
        or len(incoming_session) != 16
        or current_sequence == 0
        or incoming_sequence == 0
        or current_version == 0
        or incoming_version == 0
        or len(current_identity) != 32
        or len(incoming_identity) != 32
    ):
        raise ValueError("SEQUENCE_REJECTED: invalid stream watermark")
    if incoming_session != current_session:
        if incoming_session in retired_sessions:
            raise ValueError("SEQUENCE_REJECTED: retired producer session")
        if incoming_version < current_version:
            raise ValueError("VERSION_INCOMPATIBLE: business version rollback")
        if incoming_version == current_version and not hmac.compare_digest(
            incoming_identity, current_identity
        ):
            raise ValueError("INPUT_INVALID: cross-session identity conflict")
        retired_sessions.add(current_session)
        return "new session"
    if recovery_boundary:
        raise ValueError("SEQUENCE_REJECTED: recovery requires a new session")
    if incoming_sequence < current_sequence:
        raise ValueError("SEQUENCE_REJECTED: message reorder")
    if incoming_sequence == current_sequence:
        if incoming_version == current_version and hmac.compare_digest(
            incoming_identity, current_identity
        ):
            return "idempotent duplicate"
        raise ValueError("INPUT_INVALID: sequence identity conflict")
    if incoming_version < current_version:
        raise ValueError("VERSION_INCOMPATIBLE: business version rollback")
    if incoming_version == current_version:
        raise ValueError("INPUT_INVALID: new delivery reused business version")
    return "accepted"


class ScoutCoordinationContractTests(unittest.TestCase):
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
        cls.cooperation = importlib.import_module(
            "underwater.contracts.v1.cooperation_pb2"
        )
        cls.sensing = importlib.import_module("underwater.contracts.v1.sensing_pb2")
        cls.state = importlib.import_module("underwater.contracts.v1.state_pb2")
        cls.profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        cls.accepted_manifest_identity = b"a" * 32

    @classmethod
    def tearDownClass(cls) -> None:
        sys.path.remove(cls.generated.name)
        cls.generated.cleanup()

    def test_schema_exposes_prediction_and_coordination_as_distinct_streams(self) -> None:
        self.assertGreater(self.common.STREAM_MAIN_ROBOT_PREDICTION, 0)
        self.assertGreater(self.common.STREAM_SCOUT_COORDINATION_CONSTRAINT, 0)
        prediction_fields = {
            field.name
            for field in self.cooperation.MainRobotPrediction.DESCRIPTOR.fields
        }
        coordination_fields = {
            field.name
            for field in self.cooperation.ScoutCoordinationConstraint.DESCRIPTOR.fields
        }
        mission_identity = {
            "mission_id",
            "mission_version",
            "mission_content_identity",
        }
        self.assertTrue(mission_identity.issubset(prediction_fields))
        self.assertTrue(mission_identity.issubset(coordination_fields))
        self.assertNotIn("execution_lease", prediction_fields)
        self.assertNotIn("execution_lease", coordination_fields)

    def test_moving_prediction_round_trip_preserves_intervals_uncertainty_and_validity(self) -> None:
        prediction = self.cooperation.MainRobotPrediction(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="main-robot-prediction-authority",
                producer_session_id=b"0123456789abcdef",
                stream_id=self.common.STREAM_MAIN_ROBOT_PREDICTION,
                sequence=17,
                source_clock_domain_id="main-nuc/boot-7",
                generated_at_monotonic_ns=1_000_000_000,
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            prediction_id="main-robot/prediction/91",
            main_robot_id="laying-robot-1",
            prediction_version=91,
            alignment_epoch=self.common.SynchronizedObservationTime(
                utc_time_ns=1_789_000_000_000_000_000,
                status=self.common.TIME_SYNC_SYNCHRONIZED,
                uncertainty_ns=2_000_000,
            ),
            source_valid_from_monotonic_ns=1_000_000_000,
            source_valid_until_monotonic_ns=1_500_000_000,
            occupied_intervals=[
                self.cooperation.MainRobotOccupiedInterval(
                    start_offset_ns=0,
                    end_offset_ns=250_000_000,
                    swept_volume=self.cooperation.ConservativeSweptSphere3dEnu(
                        start_center=self.sensing.Point3dEnu(
                            x_m=0.0, y_m=0.0, z_m=-4.0, frame_id="mission_enu"
                        ),
                        end_center=self.sensing.Point3dEnu(
                            x_m=0.5, y_m=0.0, z_m=-4.0, frame_id="mission_enu"
                        ),
                        physical_radius_m=1.2,
                        position_uncertainty_radius_m=0.3,
                        conservative_occupied_radius_m=1.6,
                    ),
                ),
                self.cooperation.MainRobotOccupiedInterval(
                    start_offset_ns=250_000_000,
                    end_offset_ns=500_000_000,
                    swept_volume=self.cooperation.ConservativeSweptSphere3dEnu(
                        start_center=self.sensing.Point3dEnu(
                            x_m=0.5, y_m=0.0, z_m=-4.0, frame_id="mission_enu"
                        ),
                        end_center=self.sensing.Point3dEnu(
                            x_m=1.0, y_m=0.2, z_m=-4.0, frame_id="mission_enu"
                        ),
                        physical_radius_m=1.2,
                        position_uncertainty_radius_m=0.35,
                        conservative_occupied_radius_m=1.7,
                    ),
                ),
            ],
            prediction_content_identity=self.common.ContentIdentity(sha256=b"p" * 32),
        )

        decoded = self.cooperation.MainRobotPrediction.FromString(
            prediction.SerializeToString(deterministic=True)
        )
        self.assertEqual(decoded.prediction_version, 91)
        self.assertEqual(len(decoded.occupied_intervals), 2)
        self.assertEqual(decoded.occupied_intervals[0].end_offset_ns, 250_000_000)
        self.assertEqual(
            decoded.occupied_intervals[1].swept_volume.end_center.y_m, 0.2
        )
        self.assertEqual(
            decoded.occupied_intervals[1].swept_volume.position_uncertainty_radius_m,
            0.35,
        )
        self.assertEqual(decoded.prediction_content_identity.sha256, b"p" * 32)

    def test_coordination_round_trip_binds_prediction_limits_and_both_loss_policies(self) -> None:
        constraint = self.cooperation.ScoutCoordinationConstraint(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="main-robot-coordination-authority",
                producer_session_id=b"0123456789abcdef",
                stream_id=self.common.STREAM_SCOUT_COORDINATION_CONSTRAINT,
                sequence=23,
                source_clock_domain_id="main-nuc/boot-7",
                generated_at_monotonic_ns=1_010_000_000,
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            coordination_version=11,
            prediction_id="main-robot/prediction/91",
            prediction_version=91,
            prediction_content_identity=self.common.ContentIdentity(sha256=b"p" * 32),
            channel_id=self.state.CHANNEL_MAIN_SCOUT_COOP,
            minimum_separation_m=2.5,
            maximum_communication_distance_m=80.0,
            link_assurance_basis=(
                self.cooperation.LINK_ASSURANCE_GEOMETRIC_DISTANCE_ONLY
            ),
            scout_loss_policy=self.common.ProfileRef(
                profile_id="scout-main-coop-loss/integration-v1",
                version=1,
                content_identity=self.common.ContentIdentity(sha256=b"s" * 32),
            ),
            main_loss_policy=self.common.ProfileRef(
                profile_id="main-scout-coop-loss/integration-v1",
                version=1,
                content_identity=self.common.ContentIdentity(sha256=b"l" * 32),
            ),
            source_valid_from_monotonic_ns=1_010_000_000,
            source_valid_until_monotonic_ns=1_500_000_000,
            coordination_content_identity=self.common.ContentIdentity(sha256=b"c" * 32),
        )

        decoded = self.cooperation.ScoutCoordinationConstraint.FromString(
            constraint.SerializeToString(deterministic=True)
        )
        self.assertEqual(decoded.prediction_version, 91)
        self.assertEqual(decoded.minimum_separation_m, 2.5)
        self.assertEqual(decoded.maximum_communication_distance_m, 80.0)
        self.assertEqual(
            decoded.link_assurance_basis,
            self.cooperation.LINK_ASSURANCE_GEOMETRIC_DISTANCE_ONLY,
        )
        self.assertFalse(decoded.HasField("calibrated_link_model"))
        self.assertEqual(decoded.scout_loss_policy.content_identity.sha256, b"s" * 32)
        self.assertEqual(decoded.main_loss_policy.content_identity.sha256, b"l" * 32)

    def _header(self, *, stream_id: int, sequence: int = 1) -> object:
        return self.common.MessageHeader(
            schema_major=1,
            schema_minor=0,
            producer_id="main-coordination-authority",
            producer_session_id=b"0123456789abcdef",
            stream_id=stream_id,
            sequence=sequence,
            source_clock_domain_id="main-nuc/boot-7",
            generated_at_monotonic_ns=1_000_000_000,
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

    def _prediction(self) -> object:
        prediction = self.cooperation.MainRobotPrediction(
            header=self._header(
                stream_id=self.common.STREAM_MAIN_ROBOT_PREDICTION,
                sequence=31,
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            prediction_id="main-robot/prediction/91",
            main_robot_id="laying-robot-1",
            prediction_version=91,
            alignment_epoch=self.common.SynchronizedObservationTime(
                utc_time_ns=1_789_000_000_000_000_000,
                status=self.common.TIME_SYNC_SYNCHRONIZED,
                uncertainty_ns=2_000_000,
            ),
            source_valid_from_monotonic_ns=1_000_000_000,
            source_valid_until_monotonic_ns=1_500_000_000,
            occupied_intervals=[
                self.cooperation.MainRobotOccupiedInterval(
                    start_offset_ns=0,
                    end_offset_ns=250_000_000,
                    swept_volume=self.cooperation.ConservativeSweptSphere3dEnu(
                        start_center=self.sensing.Point3dEnu(
                            x_m=0.0, y_m=0.0, z_m=-4.0, frame_id="mission_enu"
                        ),
                        end_center=self.sensing.Point3dEnu(
                            x_m=0.5, y_m=0.0, z_m=-4.0, frame_id="mission_enu"
                        ),
                        physical_radius_m=1.2,
                        position_uncertainty_radius_m=0.3,
                        conservative_occupied_radius_m=1.6,
                    ),
                ),
                self.cooperation.MainRobotOccupiedInterval(
                    start_offset_ns=250_000_000,
                    end_offset_ns=500_000_000,
                    swept_volume=self.cooperation.ConservativeSweptSphere3dEnu(
                        start_center=self.sensing.Point3dEnu(
                            x_m=0.5, y_m=0.0, z_m=-4.0, frame_id="mission_enu"
                        ),
                        end_center=self.sensing.Point3dEnu(
                            x_m=1.0, y_m=0.2, z_m=-4.0, frame_id="mission_enu"
                        ),
                        physical_radius_m=1.2,
                        position_uncertainty_radius_m=0.35,
                        conservative_occupied_radius_m=1.7,
                    ),
                ),
            ],
        )
        prediction.prediction_content_identity.sha256 = canonical_business_identity(
            prediction, "prediction_content_identity"
        )
        return prediction

    def _coordination(self, prediction: object | None = None) -> object:
        prediction = prediction or self._prediction()
        constraint = self.cooperation.ScoutCoordinationConstraint(
            header=self._header(
                stream_id=self.common.STREAM_SCOUT_COORDINATION_CONSTRAINT,
                sequence=41,
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            coordination_version=11,
            prediction_id=prediction.prediction_id,
            prediction_version=prediction.prediction_version,
            prediction_content_identity=prediction.prediction_content_identity,
            channel_id=self.state.CHANNEL_MAIN_SCOUT_COOP,
            minimum_separation_m=2.5,
            maximum_communication_distance_m=80.0,
            link_assurance_basis=(
                self.cooperation.LINK_ASSURANCE_GEOMETRIC_DISTANCE_ONLY
            ),
            scout_loss_policy=self.common.ProfileRef(
                profile_id="scout-main-coop-loss/integration-v1",
                version=1,
                content_identity=self.common.ContentIdentity(sha256=b"s" * 32),
            ),
            main_loss_policy=self.common.ProfileRef(
                profile_id="main-scout-coop-loss/integration-v1",
                version=1,
                content_identity=self.common.ContentIdentity(sha256=b"l" * 32),
            ),
            source_valid_from_monotonic_ns=1_000_000_000,
            source_valid_until_monotonic_ns=1_500_000_000,
        )
        constraint.coordination_content_identity.sha256 = canonical_business_identity(
            constraint, "coordination_content_identity"
        )
        return constraint

    def _validate_context(
        self,
        prediction: object,
        constraint: object,
        *,
        now_local_ns: int = 2_100_000_000,
        plan_end_offset_ns: int = 500_000_000,
        mission_identity: tuple[int, int, bytes] = (73, 4, b"m" * 32),
    ) -> None:
        validate_coordination_context(
            prediction,
            constraint,
            self.common,
            self.cooperation,
            self.state,
            mission_identity=mission_identity,
            expected_coordination_version=11,
            received_prediction_at_local_ns=2_000_000_000,
            received_coordination_at_local_ns=2_010_000_000,
            now_local_ns=now_local_ns,
            plan_end_offset_ns=plan_end_offset_ns,
            profile=self.profile,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
        )

    def test_complete_context_is_accepted_for_its_closed_prediction_horizon(self) -> None:
        prediction = self._prediction()
        constraint = self._coordination(prediction)
        validate_coordination_context(
            prediction,
            constraint,
            self.common,
            self.cooperation,
            self.state,
            mission_identity=(73, 4, b"m" * 32),
            expected_coordination_version=11,
            received_prediction_at_local_ns=2_000_000_000,
            received_coordination_at_local_ns=2_010_000_000,
            now_local_ns=2_100_000_000,
            plan_end_offset_ns=500_000_000,
            profile=self.profile,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
        )

    def test_moving_occupancy_checks_inclusive_separation_communication_and_terminal_boundaries(self) -> None:
        prediction = self._prediction()
        constraint = self._coordination(prediction)

        self.assertTrue(
            coordination_allows_position(
                prediction,
                constraint,
                offset_ns=125_000_000,
                scout_position=(4.35, 0.0, -4.0),
            )
        )
        self.assertFalse(
            coordination_allows_position(
                prediction,
                constraint,
                offset_ns=125_000_000,
                scout_position=(4.349, 0.0, -4.0),
            )
        )
        self.assertFalse(
            coordination_allows_position(
                prediction,
                constraint,
                offset_ns=125_000_000,
                scout_position=(80.251, 0.0, -4.0),
            )
        )
        self.assertIsInstance(
            occupied_sphere_at_offset(prediction, 500_000_000), tuple
        )
        with self.assertRaisesRegex(ValueError, "horizon"):
            occupied_sphere_at_offset(prediction, 500_000_001)

    def test_loss_recovery_requires_new_watermarks_for_both_streams(self) -> None:
        prediction = self._prediction()
        constraint = self._coordination(prediction)
        retired_prediction_sessions: set[bytes] = set()
        retired_coordination_sessions: set[bytes] = set()

        recovered_prediction = self._prediction()
        recovered_prediction.header.producer_session_id = b"fedcba9876543210"
        recovered_prediction.header.sequence = 1
        recovered_prediction.prediction_content_identity.sha256 = canonical_business_identity(
            recovered_prediction, "prediction_content_identity"
        )
        self.assertEqual(
            apply_stream_watermark(
                current_session=prediction.header.producer_session_id,
                current_sequence=prediction.header.sequence,
                current_version=prediction.prediction_version,
                current_identity=prediction.prediction_content_identity.sha256,
                incoming_session=recovered_prediction.header.producer_session_id,
                incoming_sequence=recovered_prediction.header.sequence,
                incoming_version=recovered_prediction.prediction_version,
                incoming_identity=recovered_prediction.prediction_content_identity.sha256,
                retired_sessions=retired_prediction_sessions,
                recovery_boundary=True,
            ),
            "new session",
        )
        with self.assertRaisesRegex(ValueError, "new session"):
            apply_stream_watermark(
                current_session=constraint.header.producer_session_id,
                current_sequence=constraint.header.sequence,
                current_version=constraint.coordination_version,
                current_identity=constraint.coordination_content_identity.sha256,
                incoming_session=constraint.header.producer_session_id,
                incoming_sequence=constraint.header.sequence,
                incoming_version=constraint.coordination_version,
                incoming_identity=constraint.coordination_content_identity.sha256,
                retired_sessions=retired_coordination_sessions,
                recovery_boundary=True,
            )

        recovered_constraint = self._coordination(recovered_prediction)
        recovered_constraint.header.producer_session_id = b"fedcba9876543210"
        recovered_constraint.header.sequence = 1
        recovered_constraint.coordination_content_identity.sha256 = canonical_business_identity(
            recovered_constraint, "coordination_content_identity"
        )
        self.assertEqual(
            apply_stream_watermark(
                current_session=constraint.header.producer_session_id,
                current_sequence=constraint.header.sequence,
                current_version=constraint.coordination_version,
                current_identity=constraint.coordination_content_identity.sha256,
                incoming_session=recovered_constraint.header.producer_session_id,
                incoming_sequence=recovered_constraint.header.sequence,
                incoming_version=recovered_constraint.coordination_version,
                incoming_identity=recovered_constraint.coordination_content_identity.sha256,
                retired_sessions=retired_coordination_sessions,
                recovery_boundary=True,
            ),
            "new session",
        )
        with self.assertRaisesRegex(ValueError, "retired"):
            apply_stream_watermark(
                current_session=recovered_prediction.header.producer_session_id,
                current_sequence=recovered_prediction.header.sequence,
                current_version=recovered_prediction.prediction_version,
                current_identity=recovered_prediction.prediction_content_identity.sha256,
                incoming_session=prediction.header.producer_session_id,
                incoming_sequence=prediction.header.sequence + 1,
                incoming_version=prediction.prediction_version + 1,
                incoming_identity=b"n" * 32,
                retired_sessions=retired_prediction_sessions,
                recovery_boundary=False,
            )

    def test_prediction_gap_staleness_and_horizon_overrun_deny_new_exploration(self) -> None:
        prediction = self._prediction()
        constraint = self._coordination(prediction)
        prediction.occupied_intervals[1].start_offset_ns += 1
        prediction.prediction_content_identity.sha256 = canonical_business_identity(
            prediction, "prediction_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "gap"):
            self._validate_context(prediction, constraint)

        prediction = self._prediction()
        constraint = self._coordination(prediction)
        with self.assertRaisesRegex(ValueError, "horizon"):
            self._validate_context(
                prediction, constraint, plan_end_offset_ns=500_000_001
            )
        with self.assertRaisesRegex(ValueError, "STALE"):
            self._validate_context(
                prediction,
                constraint,
                now_local_ns=(
                    2_000_000_000
                    + self.profile["timing"]["main_robot_prediction_reject_ns"]
                    + 1
                ),
            )

    def test_clock_domain_task_and_prediction_pairing_mismatches_fail_closed(self) -> None:
        prediction = self._prediction()
        wrong_clock = self._coordination(prediction)
        wrong_clock.header.source_clock_domain_id = "main-nuc/boot-8"
        with self.assertRaisesRegex(ValueError, "CLOCK_DOMAIN_MISMATCH"):
            self._validate_context(prediction, wrong_clock)

        wrong_task = self._coordination(prediction)
        wrong_task.mission_id = 74
        wrong_task.coordination_content_identity.sha256 = canonical_business_identity(
            wrong_task, "coordination_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "task or prediction pairing"):
            self._validate_context(prediction, wrong_task)

        wrong_prediction = self._coordination(prediction)
        wrong_prediction.prediction_version += 1
        wrong_prediction.coordination_content_identity.sha256 = canonical_business_identity(
            wrong_prediction, "coordination_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "task or prediction pairing"):
            self._validate_context(prediction, wrong_prediction)

    def test_conflicting_limits_and_link_assurance_overclaims_are_rejected(self) -> None:
        prediction = self._prediction()
        conflict = self._coordination(prediction)
        conflict.maximum_communication_distance_m = conflict.minimum_separation_m
        conflict.coordination_content_identity.sha256 = canonical_business_identity(
            conflict, "coordination_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "limits or policy"):
            self._validate_context(prediction, conflict)

        overclaim = self._coordination(prediction)
        overclaim.calibrated_link_model.CopyFrom(
            self.common.ProfileRef(
                profile_id="unapproved-link-model",
                version=1,
                content_identity=self.common.ContentIdentity(sha256=b"q" * 32),
            )
        )
        overclaim.coordination_content_identity.sha256 = canonical_business_identity(
            overclaim, "coordination_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "limits or policy"):
            self._validate_context(prediction, overclaim)

    def test_alignment_nonfinite_identity_unknown_and_resource_errors_fail_closed(self) -> None:
        unsynchronized = self._prediction()
        unsynchronized.alignment_epoch.status = self.common.TIME_SYNC_DEGRADED
        unsynchronized.prediction_content_identity.sha256 = canonical_business_identity(
            unsynchronized, "prediction_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "CLOCK_DOMAIN_MISMATCH"):
            self._validate_context(unsynchronized, self._coordination(unsynchronized))

        nonfinite = self._prediction()
        nonfinite.occupied_intervals[0].swept_volume.physical_radius_m = math.nan
        nonfinite.prediction_content_identity.sha256 = canonical_business_identity(
            nonfinite, "prediction_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "occupied volume"):
            self._validate_context(nonfinite, self._coordination(nonfinite))

        tampered = self._prediction()
        constraint = self._coordination(tampered)
        tampered.main_robot_id = "laying-robot-tampered"
        with self.assertRaisesRegex(ValueError, "content identity"):
            self._validate_context(tampered, constraint)

        unknown = self.cooperation.MainRobotPrediction.FromString(
            self._prediction().SerializeToString(deterministic=True) + b"\xa0\x06\x01"
        )
        with self.assertRaisesRegex(ValueError, "unknown"):
            self._validate_context(unknown, self._coordination(unknown))

        oversized = self._prediction()
        oversized.prediction_id = "x" * (
            self.profile["interface_limits"]["maximum_string_bytes"] + 1
        )
        oversized.prediction_content_identity.sha256 = canonical_business_identity(
            oversized, "prediction_content_identity"
        )
        with self.assertRaisesRegex(ValueError, "RESOURCE_LIMIT_EXCEEDED"):
            self._validate_context(oversized, self._coordination(oversized))

    def test_stream_watermarks_reject_rollback_reuse_and_identity_conflict(self) -> None:
        session = b"0123456789abcdef"
        with self.assertRaisesRegex(ValueError, "rollback"):
            apply_stream_watermark(
                current_session=session,
                current_sequence=31,
                current_version=91,
                current_identity=b"p" * 32,
                incoming_session=session,
                incoming_sequence=32,
                incoming_version=90,
                incoming_identity=b"o" * 32,
                retired_sessions=set(),
                recovery_boundary=False,
            )
        with self.assertRaisesRegex(ValueError, "reused"):
            apply_stream_watermark(
                current_session=session,
                current_sequence=31,
                current_version=91,
                current_identity=b"p" * 32,
                incoming_session=session,
                incoming_sequence=32,
                incoming_version=91,
                incoming_identity=b"p" * 32,
                retired_sessions=set(),
                recovery_boundary=False,
            )
        with self.assertRaisesRegex(ValueError, "identity conflict"):
            apply_stream_watermark(
                current_session=session,
                current_sequence=31,
                current_version=91,
                current_identity=b"p" * 32,
                incoming_session=session,
                incoming_sequence=31,
                incoming_version=91,
                incoming_identity=b"q" * 32,
                retired_sessions=set(),
                recovery_boundary=False,
            )

    def test_normative_docs_profile_hashing_manifest_and_adapters_are_traceable(self) -> None:
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        hashing = HASHING_PATH.read_text(encoding="utf-8")
        for fragment in (
            "MainRobotPrediction",
            "ScoutCoordinationConstraint",
            "SynchronizedObservationTime",
            "source_clock_domain_id",
            "LossPolicy",
            "DEPENDENCY_STALE",
            "CLOCK_DOMAIN_MISMATCH",
            "ROS 2",
            "C++",
            "closed",
            "new producer session",
        ):
            self.assertIn(fragment, contract)
        self.assertRegex(contract, r"MUST NOT[^\n]*(lease|deadline|watchdog)")
        self.assertIn("MainRobotPrediction", hashing)
        self.assertIn("prediction_content_identity", hashing)
        self.assertIn("coordination_content_identity", hashing)

        timing = self.profile["timing"]
        limits = self.profile["interface_limits"]
        self.assertLess(
            timing["main_robot_prediction_publish_period_ns"],
            timing["main_robot_prediction_reject_ns"],
        )
        self.assertLess(
            timing["coordination_constraint_publish_period_ns"],
            timing["coordination_constraint_reject_ns"],
        )
        for key in (
            "coordination_maximum_sync_uncertainty_ns",
            "main_robot_prediction_reject_ns",
            "coordination_constraint_reject_ns",
        ):
            self.assertGreater(timing[key], 0)
        for key in (
            "maximum_main_robot_prediction_bytes",
            "maximum_prediction_intervals",
            "maximum_coordination_constraint_bytes",
        ):
            self.assertGreater(limits[key], 0)
        self.assertIn(
            "scout_main_robot_coordination_v1",
            self.manifest["supported_features"],
        )


if __name__ == "__main__":
    unittest.main()
