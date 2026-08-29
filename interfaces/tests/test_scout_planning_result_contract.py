"""Public-contract tests for immutable Scout 4D planning results."""

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
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
CONTRACT_PATH = INTERFACES / "SCOUT_4D_PLANNING_RESULT.md"
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


def canonical_business_identity(
    message: object, identity_field: str, *, exclude_header: bool = False
) -> bytes:
    canonical = type(message)()
    canonical.CopyFrom(message)
    if exclude_header:
        canonical.ClearField("header")
    canonical.ClearField(identity_field)
    _normalize_canonical_message(canonical)
    return hashlib.sha256(canonical.SerializeToString(deterministic=True)).digest()


def select_primary_outcome(outcomes: set[int], codes: object) -> int:
    priority = (
        codes.OUTCOME_INPUT_INVALID,
        codes.OUTCOME_DEPENDENCY_STALE,
        codes.OUTCOME_CAPABILITY_INFEASIBLE,
        codes.OUTCOME_ENERGY_INSUFFICIENT,
        codes.OUTCOME_COORDINATION_INFEASIBLE,
        codes.OUTCOME_NO_SOLUTION,
        codes.OUTCOME_SMOOTHING_FAILED,
        codes.OUTCOME_SURVEY_INFEASIBLE,
        codes.OUTCOME_VALIDATION_REJECTED,
        codes.OUTCOME_VALIDATION_INCONCLUSIVE,
        codes.OUTCOME_TIMEOUT,
        codes.OUTCOME_CANCELLED,
        codes.OUTCOME_NUMERICALLY_INVALID,
    )
    for outcome in priority:
        if outcome in outcomes:
            return outcome
    raise ValueError("INPUT_INVALID: no terminal failure outcome")


def apply_result_watermark(
    *,
    current_session: bytes,
    current_delivery_sequence: int,
    current_result_sequence: int,
    current_identity: bytes,
    incoming_session: bytes,
    incoming_delivery_sequence: int,
    incoming_result_sequence: int,
    incoming_identity: bytes,
    retired_sessions: set[bytes],
    recovery_boundary: bool,
) -> str:
    if incoming_session in retired_sessions:
        raise ValueError("SEQUENCE_REJECTED: retired producer session")
    if incoming_session != current_session:
        if not recovery_boundary:
            raise ValueError("SEQUENCE_REJECTED: new session requires recovery boundary")
        retired_sessions.add(current_session)
        return "new session"
    if incoming_delivery_sequence < current_delivery_sequence:
        raise ValueError("SEQUENCE_REJECTED: delivery rollback")
    if incoming_delivery_sequence == current_delivery_sequence:
        if (
            incoming_result_sequence == current_result_sequence
            and hmac.compare_digest(incoming_identity, current_identity)
        ):
            return "idempotent duplicate"
        raise ValueError("INPUT_INVALID: delivery identity conflict")
    if incoming_result_sequence < current_result_sequence:
        raise ValueError("SEQUENCE_REJECTED: result rollback")
    if incoming_result_sequence == current_result_sequence:
        if not hmac.compare_digest(incoming_identity, current_identity):
            raise ValueError("INPUT_INVALID: result identity conflict")
        raise ValueError("SEQUENCE_REJECTED: result sequence reused")
    if hmac.compare_digest(incoming_identity, current_identity):
        raise ValueError("SEQUENCE_REJECTED: content identity reused")
    return "newer result"


def _has_unknown_fields(message: object) -> bool:
    known_only = type(message)()
    known_only.CopyFrom(message)
    known_only.DiscardUnknownFields()
    return known_only.SerializeToString(deterministic=True) != message.SerializeToString(
        deterministic=True
    )


def _message_scalars(message: object) -> tuple[list[str], list[float]]:
    strings: list[str] = []
    floats: list[float] = []
    for field, value in message.ListFields():
        if field.label == FieldDescriptor.LABEL_REPEATED:
            if field.type == FieldDescriptor.TYPE_MESSAGE:
                for item in value:
                    nested_strings, nested_floats = _message_scalars(item)
                    strings.extend(nested_strings)
                    floats.extend(nested_floats)
            elif field.type == FieldDescriptor.TYPE_STRING:
                strings.extend(value)
            elif field.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
                floats.extend(value)
        elif field.type == FieldDescriptor.TYPE_MESSAGE:
            nested_strings, nested_floats = _message_scalars(value)
            strings.extend(nested_strings)
            floats.extend(nested_floats)
        elif field.type == FieldDescriptor.TYPE_STRING:
            strings.append(value)
        elif field.type in (FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.TYPE_FLOAT):
            floats.append(value)
    return strings, floats


def _profile_ref_is_valid(reference: object) -> bool:
    return (
        bool(reference.profile_id)
        and reference.version > 0
        and len(reference.content_identity.sha256) == 32
    )


def _identity_is_valid(identity: object) -> bool:
    return len(identity.sha256) == 32


def _validate_complete_dependencies(dependencies: object) -> None:
    sensors = dependencies.sensors
    sensor_ids = [sensor.sensor_id for sensor in sensors]
    if (
        dependencies.mission_id == 0
        or dependencies.mission_version == 0
        or not _identity_is_valid(dependencies.mission_content_identity)
        or not dependencies.map_id
        or dependencies.map_version == 0
        or not _identity_is_valid(dependencies.map_content_identity)
        or dependencies.navigation_version == 0
        or not _identity_is_valid(dependencies.navigation_content_identity)
        or not sensors
        or sensor_ids != sorted(set(sensor_ids))
        or any(
            not sensor.sensor_id
            or sensor.geometry_version == 0
            or not _identity_is_valid(sensor.geometry_content_identity)
            or sensor.health_version == 0
            or not _identity_is_valid(sensor.health_content_identity)
            for sensor in sensors
        )
        or not dependencies.current_model_id
        or dependencies.current_model_version == 0
        or not _identity_is_valid(dependencies.current_content_identity)
        or not _profile_ref_is_valid(dependencies.capability_profile)
        or dependencies.thruster_health_version == 0
        or not _identity_is_valid(dependencies.thruster_health_content_identity)
        or not _profile_ref_is_valid(dependencies.energy_model)
        or not dependencies.energy_store_id
        or dependencies.energy_state_version == 0
        or not _identity_is_valid(dependencies.energy_state_content_identity)
        or not dependencies.prediction_id
        or dependencies.prediction_version == 0
        or not _identity_is_valid(dependencies.prediction_content_identity)
        or dependencies.coordination_version == 0
        or not _identity_is_valid(dependencies.coordination_content_identity)
        or not _profile_ref_is_valid(dependencies.planner_configuration)
        or not _profile_ref_is_valid(dependencies.timing_profile)
        or not _profile_ref_is_valid(dependencies.interface_limits)
        or not hmac.compare_digest(
            dependencies.dependencies_content_identity.sha256,
            canonical_business_identity(
                dependencies, "dependencies_content_identity"
            ),
        )
    ):
        raise ValueError("INPUT_INVALID: incomplete planning dependencies")


def _validate_candidate(
    candidate: object,
    result_dependencies: object,
    codes: object,
    planning: object,
    *,
    evaluated_at_monotonic_ns: int,
    maximum_segments: int,
) -> None:
    if (
        candidate.plan_sequence == 0
        or not candidate.HasField("created_at_monotonic_ns")
        or candidate.created_at_monotonic_ns < 0
        or candidate.created_at_monotonic_ns > evaluated_at_monotonic_ns
    ):
        raise ValueError("INPUT_INVALID: candidate identity or creation time")
    _validate_complete_dependencies(candidate.dependencies)
    if (
        candidate.dependencies.SerializeToString(deterministic=True)
        != result_dependencies.SerializeToString(deterministic=True)
    ):
        raise ValueError("INPUT_INVALID: candidate/result dependencies differ")

    validate_trajectory(candidate.trajectory, maximum_segments=maximum_segments)
    if not hmac.compare_digest(
        candidate.trajectory.trajectory_content_identity.sha256,
        canonical_business_identity(
            candidate.trajectory, "trajectory_content_identity"
        ),
    ):
        raise ValueError("INPUT_INVALID: trajectory content identity")

    evidence = candidate.survey_evidence
    if (
        evidence.mission_id != candidate.dependencies.mission_id
        or evidence.mission_version != candidate.dependencies.mission_version
        or not hmac.compare_digest(
            evidence.mission_content_identity.sha256,
            candidate.dependencies.mission_content_identity.sha256,
        )
        or evidence.baseline_map_id != candidate.dependencies.map_id
        or evidence.baseline_map_version != candidate.dependencies.map_version
        or not hmac.compare_digest(
            evidence.baseline_map_content_identity.sha256,
            candidate.dependencies.map_content_identity.sha256,
        )
        or not hmac.compare_digest(
            evidence.evidence_content_identity.sha256,
            canonical_business_identity(evidence, "evidence_content_identity"),
        )
    ):
        raise ValueError("INPUT_INVALID: survey evidence binding")

    report = candidate.validation_report
    if (
        report.status != planning.SCOUT_PLAN_VALIDATION_SAFE
        or report.primary_outcome != codes.OUTCOME_SUCCESS
    ):
        raise ValueError("INPUT_INVALID: candidate requires a SAFE validation report")
    required_safe_metrics = (
        "minimum_collision_margin_m",
        "minimum_separation_margin_m",
        "minimum_energy_margin_j",
        "minimum_capability_margin",
        "survey_coverage_ratio",
    )
    if (
        report.HasField("earliest_failure_time_offset_ns")
        or not all(report.HasField(field) for field in required_safe_metrics)
        or not all(
            math.isfinite(getattr(report, field)) for field in required_safe_metrics
        )
        or report.minimum_collision_margin_m < 0.0
        or report.minimum_separation_margin_m < 0.0
        or report.minimum_energy_margin_j < 0.0
        or report.minimum_capability_margin < 0.0
        or not 0.0 <= report.survey_coverage_ratio <= 1.0
    ):
        raise ValueError("INPUT_INVALID: SAFE report requires complete finite metrics")
    report_bindings = (
        (
            report.validated_dependencies_content_identity.sha256,
            candidate.dependencies.dependencies_content_identity.sha256,
        ),
        (
            report.validated_trajectory_content_identity.sha256,
            candidate.trajectory.trajectory_content_identity.sha256,
        ),
        (
            report.validated_survey_evidence_content_identity.sha256,
            evidence.evidence_content_identity.sha256,
        ),
    )
    if any(not hmac.compare_digest(left, right) for left, right in report_bindings):
        raise ValueError("INPUT_INVALID: validation report input binding")
    if not hmac.compare_digest(
        report.validation_report_content_identity.sha256,
        canonical_business_identity(report, "validation_report_content_identity"),
    ):
        raise ValueError("INPUT_INVALID: validation report content identity")
    if not hmac.compare_digest(
        candidate.plan_content_identity.sha256,
        canonical_business_identity(candidate, "plan_content_identity"),
    ):
        raise ValueError("INPUT_INVALID: plan content identity")


def validate_result(
    result: object,
    common: object,
    codes: object,
    planning: object,
    *,
    manifest: dict[str, object],
    accepted_manifest_identity: bytes,
    maximum_segments: int,
    maximum_diagnostics: int,
    maximum_bytes: int,
) -> None:
    allowed_outcomes = {
        codes.OUTCOME_SUCCESS,
        codes.OUTCOME_INPUT_INVALID,
        codes.OUTCOME_DEPENDENCY_STALE,
        codes.OUTCOME_NO_SOLUTION,
        codes.OUTCOME_SMOOTHING_FAILED,
        codes.OUTCOME_CAPABILITY_INFEASIBLE,
        codes.OUTCOME_ENERGY_INSUFFICIENT,
        codes.OUTCOME_COORDINATION_INFEASIBLE,
        codes.OUTCOME_SURVEY_INFEASIBLE,
        codes.OUTCOME_VALIDATION_REJECTED,
        codes.OUTCOME_VALIDATION_INCONCLUSIVE,
        codes.OUTCOME_TIMEOUT,
        codes.OUTCOME_CANCELLED,
        codes.OUTCOME_NUMERICALLY_INVALID,
    }
    if result.outcome not in allowed_outcomes:
        raise ValueError("INPUT_INVALID: unknown outcome")
    if _has_unknown_fields(result):
        raise ValueError("INPUT_INVALID: unknown Scout planning result field")
    strings, floats = _message_scalars(result)
    if any(value != unicodedata.normalize("NFC", value) for value in strings):
        raise ValueError("INPUT_INVALID: non-NFC business string")
    if any(not math.isfinite(value) for value in floats):
        raise ValueError("INPUT_INVALID: non-finite business value")
    if (
        len(result.SerializeToString(deterministic=True)) > maximum_bytes
        or len(result.diagnostics) > maximum_diagnostics
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: Scout planning result")
    header = result.header
    if (
        header.schema_major != manifest["schema_major"]
        or header.schema_minor != manifest["schema_minor"]
        or header.stream_id != common.STREAM_SCOUT_PLANNING_RESULT
        or not header.producer_id
        or len(header.producer_session_id) != 16
        or header.sequence == 0
        or not header.source_clock_domain_id
        or not header.HasField("generated_at_monotonic_ns")
        or not header.HasField("manifest")
        or not hmac.compare_digest(
            header.manifest.manifest_identity.sha256,
            accepted_manifest_identity,
        )
        or "scout_4d_planning_result_v1" not in manifest["supported_features"]
    ):
        raise ValueError("INPUT_INVALID: header or exact Manifest feature")
    if (
        result.result_sequence == 0
        or not result.HasField("evaluated_at_monotonic_ns")
        or result.evaluated_at_monotonic_ns < 0
        or result.evaluated_at_monotonic_ns > header.generated_at_monotonic_ns
        or result.outcome == codes.OUTCOME_CODE_UNSPECIFIED
        or not hmac.compare_digest(
            result.result_content_identity.sha256,
            canonical_business_identity(
                result, "result_content_identity", exclude_header=True
            ),
        )
    ):
        raise ValueError("INPUT_INVALID: result identity, time, or outcome")
    for diagnostic in result.diagnostics:
        if (
            diagnostic.numeric_code == 0
            or diagnostic.registry_id != "underwater-system-codes"
            or diagnostic.registry_version != 1
        ):
            raise ValueError("INPUT_INVALID: structured diagnostic")

    if result.outcome == codes.OUTCOME_SUCCESS:
        if not result.HasField("candidate"):
            raise ValueError("INPUT_INVALID: SUCCESS requires candidate")
        _validate_candidate(
            result.candidate,
            result.dependencies,
            codes,
            planning,
            evaluated_at_monotonic_ns=result.evaluated_at_monotonic_ns,
            maximum_segments=maximum_segments,
        )
    else:
        if result.HasField("candidate"):
            raise ValueError("INPUT_INVALID: failure result cannot carry candidate")
        if not result.diagnostics:
            raise ValueError("INPUT_INVALID: failure result requires diagnostics")
        if (
            not hmac.compare_digest(
                result.dependencies.dependencies_content_identity.sha256,
                canonical_business_identity(
                    result.dependencies, "dependencies_content_identity"
                ),
            )
        ):
            raise ValueError("INPUT_INVALID: failure dependency summary")


def _point(point: object) -> tuple[float, float, float]:
    if not all(point.HasField(field) for field in ("x_m", "y_m", "z_m")):
        raise ValueError("INPUT_INVALID: position control point presence")
    result = (point.x_m, point.y_m, point.z_m)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("INPUT_INVALID: non-finite position control point")
    return result


def _close(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-12, abs_tol=1e-12)


def _vector_close(left: tuple[float, ...], right: tuple[float, ...]) -> bool:
    return all(_close(a, b) for a, b in zip(left, right, strict=True))


def validate_trajectory(trajectory: object, *, maximum_segments: int) -> None:
    if (
        trajectory.frame_id != "mission_enu"
        or not trajectory.HasField("initial_yaw_rad")
        or not math.isfinite(trajectory.initial_yaw_rad)
        or not (-math.pi <= trajectory.initial_yaw_rad < math.pi)
    ):
        raise ValueError("INPUT_INVALID: frame or initial yaw")
    if not trajectory.segments or len(trajectory.segments) > maximum_segments:
        raise ValueError("RESOURCE_LIMIT_EXCEEDED: trajectory segments")

    expected_start_ns = 0
    previous: object | None = None
    for segment in trajectory.segments:
        if segment.duration_ns == 0:
            raise ValueError("INPUT_INVALID: segment duration")
        if segment.start_time_offset_ns != expected_start_ns:
            raise ValueError("INPUT_INVALID: segments are not time-contiguous")
        if (
            len(segment.position_control_points) != 6
            or len(segment.yaw_offset_control_points_rad) != 6
        ):
            raise ValueError("INPUT_INVALID: quintic segment requires six controls")
        positions = [_point(point) for point in segment.position_control_points]
        yaws = tuple(segment.yaw_offset_control_points_rad)
        if not all(math.isfinite(value) for value in yaws):
            raise ValueError("INPUT_INVALID: non-finite yaw offset")
        if previous is None:
            if not _close(yaws[0], 0.0):
                raise ValueError("INPUT_INVALID: first yaw offset must be zero")
        else:
            previous_positions = [
                _point(point) for point in previous.position_control_points
            ]
            previous_yaws = tuple(previous.yaw_offset_control_points_rad)
            if not _vector_close(previous_positions[5], positions[0]):
                raise ValueError("INPUT_INVALID: position is not C0 continuous")
            if not _close(previous_yaws[5], yaws[0]):
                raise ValueError("INPUT_INVALID: yaw offset is not continuous")

            previous_duration = previous.duration_ns / 1_000_000_000.0
            duration = segment.duration_ns / 1_000_000_000.0
            previous_velocity = tuple(
                5.0 * (previous_positions[5][axis] - previous_positions[4][axis])
                / previous_duration
                for axis in range(3)
            )
            velocity = tuple(
                5.0 * (positions[1][axis] - positions[0][axis]) / duration
                for axis in range(3)
            )
            previous_acceleration = tuple(
                20.0
                * (
                    previous_positions[5][axis]
                    - 2.0 * previous_positions[4][axis]
                    + previous_positions[3][axis]
                )
                / (previous_duration * previous_duration)
                for axis in range(3)
            )
            acceleration = tuple(
                20.0
                * (positions[2][axis] - 2.0 * positions[1][axis] + positions[0][axis])
                / (duration * duration)
                for axis in range(3)
            )
            previous_yaw_rate = (
                5.0 * (previous_yaws[5] - previous_yaws[4]) / previous_duration
            )
            yaw_rate = 5.0 * (yaws[1] - yaws[0]) / duration
            previous_yaw_acceleration = (
                20.0
                * (previous_yaws[5] - 2.0 * previous_yaws[4] + previous_yaws[3])
                / (previous_duration * previous_duration)
            )
            yaw_acceleration = (
                20.0 * (yaws[2] - 2.0 * yaws[1] + yaws[0])
                / (duration * duration)
            )
            if not (
                _vector_close(previous_velocity, velocity)
                and _vector_close(previous_acceleration, acceleration)
            ):
                raise ValueError("INPUT_INVALID: position is not C2 continuous")
            if not (
                _close(previous_yaw_rate, yaw_rate)
                and _close(previous_yaw_acceleration, yaw_acceleration)
            ):
                raise ValueError("INPUT_INVALID: yaw offset is not C2 continuous")
        expected_start_ns += segment.duration_ns
        previous = segment


class ScoutPlanningResultContractTest(unittest.TestCase):
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
        cls.common = importlib.import_module(
            "underwater.contracts.v1.common_pb2"
        )
        cls.planning = importlib.import_module(
            "underwater.contracts.v1.planning_pb2"
        )
        cls.cooperation = importlib.import_module(
            "underwater.contracts.v1.cooperation_pb2"
        )
        cls.codes = importlib.import_module("underwater.contracts.v1.codes_pb2")
        cls.manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        cls.profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        cls.accepted_manifest_identity = b"a" * 32

    @classmethod
    def tearDownClass(cls) -> None:
        sys.path.remove(cls.generated_directory.name)
        cls.generated_directory.cleanup()

    def test_scout_plan_has_a_dedicated_non_authorizing_publication_surface(self) -> None:
        self.assertTrue(hasattr(self.planning, "ScoutPlan"))
        self.assertTrue(hasattr(self.planning, "ScoutPlanningResult"))
        self.assertTrue(hasattr(self.common, "STREAM_SCOUT_PLANNING_RESULT"))
        self.assertNotEqual(
            self.common.STREAM_SCOUT_PLANNING_RESULT,
            self.common.STREAM_PLANNING_RESULT,
        )

    def test_quintic_bezier_segment_round_trips_all_4d_control_values(self) -> None:
        point_type = getattr(self.planning, "ScoutBezierControlPoint3dEnu")
        segment = self.planning.ScoutBezierSegment4d(
            start_time_offset_ns=0,
            duration_ns=2_000_000_000,
            position_control_points=[
                point_type(x_m=float(index), y_m=0.5 * index, z_m=-4.0)
                for index in range(6)
            ],
            yaw_offset_control_points_rad=[0.0, 0.15, 0.3, 0.45, 0.6, 0.75],
        )
        trajectory = self.planning.ScoutTrajectory4d(
            frame_id="mission_enu",
            initial_yaw_rad=3.0,
            segments=[segment],
        )

        decoded = self.planning.ScoutTrajectory4d.FromString(
            trajectory.SerializeToString(deterministic=True)
        )
        self.assertEqual(len(decoded.segments[0].position_control_points), 6)
        self.assertEqual(
            list(decoded.segments[0].yaw_offset_control_points_rad),
            [0.0, 0.15, 0.3, 0.45, 0.6, 0.75],
        )
        self.assertEqual(decoded.segments[0].duration_ns, 2_000_000_000)

    def test_candidate_binds_every_input_evidence_and_independent_validation(self) -> None:
        identity = self.common.ContentIdentity(sha256=b"x" * 32)
        profile_ref = self.common.ProfileRef(
            profile_id="profile/v1", version=1, content_identity=identity
        )
        dependencies = self.planning.ScoutPlanningDependencies(
            mission_id=73,
            mission_version=4,
            mission_content_identity=identity,
            map_id="hybrid-map/scout",
            map_version=8,
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
            capability_profile=profile_ref,
            thruster_health_version=6,
            thruster_health_content_identity=identity,
            energy_model=profile_ref,
            energy_store_id="battery-a",
            energy_state_version=7,
            energy_state_content_identity=identity,
            prediction_id="laying-prediction",
            prediction_version=9,
            prediction_content_identity=identity,
            coordination_version=11,
            coordination_content_identity=identity,
            planner_configuration=profile_ref,
            timing_profile=profile_ref,
            interface_limits=profile_ref,
            dependencies_content_identity=identity,
        )
        evidence = self.cooperation.SurveyPlanEvidence(
            mission_id=73,
            mission_version=4,
            mission_content_identity=identity,
            baseline_map_id="hybrid-map/scout",
            baseline_map_version=8,
            baseline_map_content_identity=identity,
            conservative_predicted_coverage_ratio=0.95,
            predicted_resolution_m=0.1,
            evidence_content_identity=identity,
        )
        report = self.planning.ScoutPlanValidationReport(
            status=self.planning.SCOUT_PLAN_VALIDATION_SAFE,
            primary_outcome=self.codes.OUTCOME_SUCCESS,
            minimum_collision_margin_m=1.1,
            minimum_separation_margin_m=1.2,
            minimum_energy_margin_j=2_000.0,
            minimum_capability_margin=0.15,
            survey_coverage_ratio=0.95,
            refinement_depth=4,
            validated_dependencies_content_identity=identity,
            validated_trajectory_content_identity=identity,
            validated_survey_evidence_content_identity=identity,
            validation_report_content_identity=identity,
        )
        candidate = self.planning.ScoutPlan(
            plan_sequence=21,
            created_at_monotonic_ns=2_000_000_000,
            trajectory=self.planning.ScoutTrajectory4d(
                frame_id="mission_enu",
                initial_yaw_rad=0.0,
                trajectory_content_identity=identity,
            ),
            dependencies=dependencies,
            survey_evidence=evidence,
            validation_report=report,
            plan_content_identity=identity,
        )

        decoded = self.planning.ScoutPlan.FromString(
            candidate.SerializeToString(deterministic=True)
        )
        self.assertEqual(decoded.dependencies.sensors[0].sensor_id, "forward-sonar")
        self.assertEqual(
            decoded.validation_report.validated_survey_evidence_content_identity.sha256,
            decoded.survey_evidence.evidence_content_identity.sha256,
        )

    def test_failure_result_round_trips_all_terminal_outcomes_and_audit_payload(self) -> None:
        expected_outcomes = (
            "OUTCOME_SUCCESS",
            "OUTCOME_INPUT_INVALID",
            "OUTCOME_DEPENDENCY_STALE",
            "OUTCOME_NO_SOLUTION",
            "OUTCOME_SMOOTHING_FAILED",
            "OUTCOME_CAPABILITY_INFEASIBLE",
            "OUTCOME_ENERGY_INSUFFICIENT",
            "OUTCOME_COORDINATION_INFEASIBLE",
            "OUTCOME_SURVEY_INFEASIBLE",
            "OUTCOME_VALIDATION_REJECTED",
            "OUTCOME_VALIDATION_INCONCLUSIVE",
            "OUTCOME_TIMEOUT",
            "OUTCOME_CANCELLED",
            "OUTCOME_NUMERICALLY_INVALID",
        )
        for name in expected_outcomes:
            self.assertTrue(hasattr(self.codes, name), name)

        result = self.planning.ScoutPlanningResult(
            result_sequence=33,
            outcome=self.codes.OUTCOME_ENERGY_INSUFFICIENT,
            evaluated_at_monotonic_ns=2_500_000_000,
            dependencies=self.planning.ScoutPlanningDependencies(
                mission_id=73,
                mission_version=4,
                mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
                dependencies_content_identity=self.common.ContentIdentity(sha256=b"d" * 32),
            ),
            diagnostics=[
                self.common.CodeRef(
                    numeric_code=131075,
                    registry_id="underwater-system-codes",
                    registry_version=1,
                )
            ],
            result_content_identity=self.common.ContentIdentity(sha256=b"r" * 32),
        )
        decoded = self.planning.ScoutPlanningResult.FromString(
            result.SerializeToString(deterministic=True)
        )
        self.assertFalse(decoded.HasField("candidate"))
        self.assertEqual(decoded.evaluated_at_monotonic_ns, 2_500_000_000)
        self.assertEqual(decoded.dependencies.mission_id, 73)
        self.assertEqual(decoded.diagnostics[0].numeric_code, 131075)

    def test_yaw_crossing_is_continuous_and_malformed_segments_fail_closed(self) -> None:
        point_type = self.planning.ScoutBezierControlPoint3dEnu

        def segment(start_ns: int, first_x: int, first_yaw_offset: float) -> object:
            return self.planning.ScoutBezierSegment4d(
                start_time_offset_ns=start_ns,
                duration_ns=1_000_000_000,
                position_control_points=[
                    point_type(x_m=first_x + index, y_m=0.0, z_m=-4.0)
                    for index in range(6)
                ],
                yaw_offset_control_points_rad=[
                    first_yaw_offset + 0.04 * index for index in range(6)
                ],
            )

        trajectory = self.planning.ScoutTrajectory4d(
            frame_id="mission_enu",
            initial_yaw_rad=3.1,
            segments=[segment(0, 0, 0.0), segment(1_000_000_000, 5, 0.2)],
            trajectory_content_identity=self.common.ContentIdentity(sha256=b"t" * 32),
        )
        validate_trajectory(trajectory, maximum_segments=8)
        terminal_yaw = trajectory.initial_yaw_rad + trajectory.segments[-1].yaw_offset_control_points_rad[-1]
        displayed_yaw = (terminal_yaw + math.pi) % (2.0 * math.pi) - math.pi
        self.assertLess(displayed_yaw, -2.7)

        malformed = self.planning.ScoutTrajectory4d()
        malformed.CopyFrom(trajectory)
        malformed.segments[0].duration_ns = 0
        with self.assertRaisesRegex(ValueError, "duration"):
            validate_trajectory(malformed, maximum_segments=8)

        malformed.CopyFrom(trajectory)
        del malformed.segments[0].position_control_points[-1]
        with self.assertRaisesRegex(ValueError, "six"):
            validate_trajectory(malformed, maximum_segments=8)

        malformed.CopyFrom(trajectory)
        malformed.segments[1].start_time_offset_ns += 1
        with self.assertRaisesRegex(ValueError, "contiguous"):
            validate_trajectory(malformed, maximum_segments=8)

        malformed.CopyFrom(trajectory)
        malformed.segments[1].yaw_offset_control_points_rad[0] = 0.0
        with self.assertRaisesRegex(ValueError, "yaw"):
            validate_trajectory(malformed, maximum_segments=8)

    def test_canonical_hash_normalizes_data_but_never_wraps_yaw_offsets(self) -> None:
        point_type = self.planning.ScoutBezierControlPoint3dEnu
        trajectory = self.planning.ScoutTrajectory4d(
            frame_id="missio\u0301n_enu",
            initial_yaw_rad=-0.0,
            segments=[
                self.planning.ScoutBezierSegment4d(
                    start_time_offset_ns=0,
                    duration_ns=1,
                    position_control_points=[
                        point_type(x_m=-0.0, y_m=float(index), z_m=-4.0)
                        for index in range(6)
                    ],
                    yaw_offset_control_points_rad=[0.0, 0.1, 0.2, 0.3, 0.4, 0.5],
                )
            ],
        )
        canonical_identity = canonical_business_identity(
            trajectory, "trajectory_content_identity"
        )

        equivalent = self.planning.ScoutTrajectory4d()
        equivalent.CopyFrom(trajectory)
        equivalent.frame_id = unicodedata.normalize("NFC", trajectory.frame_id)
        equivalent.initial_yaw_rad = 0.0
        for point in equivalent.segments[0].position_control_points:
            point.x_m = 0.0
        self.assertEqual(
            canonical_identity,
            canonical_business_identity(equivalent, "trajectory_content_identity"),
        )

        wrapped_offset = self.planning.ScoutTrajectory4d()
        wrapped_offset.CopyFrom(equivalent)
        wrapped_offset.segments[0].yaw_offset_control_points_rad[-1] += 2.0 * math.pi
        self.assertNotEqual(
            canonical_identity,
            canonical_business_identity(wrapped_offset, "trajectory_content_identity"),
        )

    def test_primary_outcome_uses_the_normative_failure_priority(self) -> None:
        self.assertEqual(
            select_primary_outcome(
                {
                    self.codes.OUTCOME_TIMEOUT,
                    self.codes.OUTCOME_COORDINATION_INFEASIBLE,
                    self.codes.OUTCOME_ENERGY_INSUFFICIENT,
                },
                self.codes,
            ),
            self.codes.OUTCOME_ENERGY_INSUFFICIENT,
        )
        self.assertEqual(
            select_primary_outcome(
                {
                    self.codes.OUTCOME_CANCELLED,
                    self.codes.OUTCOME_NUMERICALLY_INVALID,
                },
                self.codes,
            ),
            self.codes.OUTCOME_CANCELLED,
        )
        with self.assertRaisesRegex(ValueError, "terminal failure"):
            select_primary_outcome({self.codes.OUTCOME_SUCCESS}, self.codes)

    def _header(self) -> object:
        return self.common.MessageHeader(
            schema_major=self.manifest["schema_major"],
            schema_minor=self.manifest["schema_minor"],
            producer_id="scout-planner",
            producer_session_id=b"0123456789abcdef",
            stream_id=self.common.STREAM_SCOUT_PLANNING_RESULT,
            sequence=41,
            source_clock_domain_id="scout-nuc/boot-7",
            generated_at_monotonic_ns=2_600_000_000,
            event_id=b"event-event-00001",
            correlation_id=b"corr--corr--0001",
            manifest=self.common.ContractManifestRef(
                schema_major=self.manifest["schema_major"],
                schema_minor=self.manifest["schema_minor"],
                manifest_identity=self.common.ContentIdentity(
                    sha256=self.accepted_manifest_identity
                ),
            ),
        )

    def _dependencies(self) -> object:
        identity = self.common.ContentIdentity(sha256=b"x" * 32)
        profile_ref = self.common.ProfileRef(
            profile_id="profile/v1", version=1, content_identity=identity
        )
        dependencies = self.planning.ScoutPlanningDependencies(
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            map_id="hybrid-map/scout",
            map_version=8,
            map_content_identity=self.common.ContentIdentity(sha256=b"b" * 32),
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
            capability_profile=profile_ref,
            thruster_health_version=6,
            thruster_health_content_identity=identity,
            energy_model=profile_ref,
            energy_store_id="battery-a",
            energy_state_version=7,
            energy_state_content_identity=identity,
            prediction_id="laying-prediction",
            prediction_version=9,
            prediction_content_identity=identity,
            coordination_version=11,
            coordination_content_identity=identity,
            planner_configuration=profile_ref,
            timing_profile=profile_ref,
            interface_limits=profile_ref,
        )
        dependencies.dependencies_content_identity.sha256 = canonical_business_identity(
            dependencies, "dependencies_content_identity"
        )
        return dependencies

    def _trajectory(self) -> object:
        point_type = self.planning.ScoutBezierControlPoint3dEnu
        trajectory = self.planning.ScoutTrajectory4d(
            frame_id="mission_enu",
            initial_yaw_rad=0.0,
            segments=[
                self.planning.ScoutBezierSegment4d(
                    start_time_offset_ns=0,
                    duration_ns=1_000_000_000,
                    position_control_points=[
                        point_type(x_m=float(index), y_m=0.0, z_m=-4.0)
                        for index in range(6)
                    ],
                    yaw_offset_control_points_rad=[0.0] * 6,
                )
            ],
        )
        trajectory.trajectory_content_identity.sha256 = canonical_business_identity(
            trajectory, "trajectory_content_identity"
        )
        return trajectory

    def _candidate(self) -> object:
        dependencies = self._dependencies()
        trajectory = self._trajectory()
        evidence = self.cooperation.SurveyPlanEvidence(
            mission_id=dependencies.mission_id,
            mission_version=dependencies.mission_version,
            mission_content_identity=dependencies.mission_content_identity,
            baseline_map_id=dependencies.map_id,
            baseline_map_version=dependencies.map_version,
            baseline_map_content_identity=dependencies.map_content_identity,
            conservative_predicted_coverage_ratio=0.95,
            predicted_resolution_m=0.1,
            predicted_covered_region=self.cooperation.Region3dEnu(
                xyz_m=[0.0, 0.0, -6.0, 6.0, 2.0, -2.0],
                frame_id="mission_enu",
            ),
        )
        evidence.evidence_content_identity.sha256 = canonical_business_identity(
            evidence, "evidence_content_identity"
        )
        report = self.planning.ScoutPlanValidationReport(
            status=self.planning.SCOUT_PLAN_VALIDATION_SAFE,
            primary_outcome=self.codes.OUTCOME_SUCCESS,
            minimum_collision_margin_m=1.1,
            minimum_separation_margin_m=1.2,
            minimum_energy_margin_j=2_000.0,
            minimum_capability_margin=0.15,
            survey_coverage_ratio=0.95,
            refinement_depth=4,
            validated_dependencies_content_identity=dependencies.dependencies_content_identity,
            validated_trajectory_content_identity=trajectory.trajectory_content_identity,
            validated_survey_evidence_content_identity=evidence.evidence_content_identity,
        )
        report.validation_report_content_identity.sha256 = canonical_business_identity(
            report, "validation_report_content_identity"
        )
        candidate = self.planning.ScoutPlan(
            plan_sequence=21,
            created_at_monotonic_ns=2_000_000_000,
            trajectory=trajectory,
            dependencies=dependencies,
            survey_evidence=evidence,
            validation_report=report,
        )
        candidate.plan_content_identity.sha256 = canonical_business_identity(
            candidate, "plan_content_identity"
        )
        return candidate

    def _result(self) -> object:
        candidate = self._candidate()
        result = self.planning.ScoutPlanningResult(
            header=self._header(),
            result_sequence=33,
            outcome=self.codes.OUTCOME_SUCCESS,
            candidate=candidate,
            evaluated_at_monotonic_ns=2_500_000_000,
            dependencies=candidate.dependencies,
        )
        result.result_content_identity.sha256 = canonical_business_identity(
            result, "result_content_identity", exclude_header=True
        )
        return result

    def test_success_candidate_and_failure_payload_gates_fail_closed(self) -> None:
        success = self._result()
        validate_result(
            success,
            self.common,
            self.codes,
            self.planning,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            maximum_segments=8,
            maximum_diagnostics=16,
            maximum_bytes=262_144,
        )

        missing_candidate = self._result()
        missing_candidate.ClearField("candidate")
        missing_candidate.result_content_identity.sha256 = canonical_business_identity(
            missing_candidate, "result_content_identity", exclude_header=True
        )
        with self.assertRaisesRegex(ValueError, "SUCCESS.*candidate"):
            validate_result(
                missing_candidate, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=8, maximum_diagnostics=16, maximum_bytes=262_144,
            )

        failed_with_candidate = self._result()
        failed_with_candidate.outcome = self.codes.OUTCOME_TIMEOUT
        failed_with_candidate.diagnostics.add(
            numeric_code=131075,
            registry_id="underwater-system-codes",
            registry_version=1,
        )
        failed_with_candidate.result_content_identity.sha256 = canonical_business_identity(
            failed_with_candidate, "result_content_identity", exclude_header=True
        )
        with self.assertRaisesRegex(ValueError, "failure.*candidate"):
            validate_result(
                failed_with_candidate, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=8, maximum_diagnostics=16, maximum_bytes=262_144,
            )

        unsafe = self._result()
        unsafe.candidate.validation_report.status = self.planning.SCOUT_PLAN_VALIDATION_UNSAFE
        unsafe.candidate.validation_report.validation_report_content_identity.sha256 = canonical_business_identity(
            unsafe.candidate.validation_report, "validation_report_content_identity"
        )
        unsafe.candidate.plan_content_identity.sha256 = canonical_business_identity(
            unsafe.candidate, "plan_content_identity"
        )
        unsafe.result_content_identity.sha256 = canonical_business_identity(
            unsafe, "result_content_identity", exclude_header=True
        )
        with self.assertRaisesRegex(ValueError, "SAFE"):
            validate_result(
                unsafe, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=8, maximum_diagnostics=16, maximum_bytes=262_144,
            )

    def test_unknown_oversized_and_tampered_results_are_rejected(self) -> None:
        limits = self.profile["interface_limits"]
        maximum_bytes = limits["maximum_scout_planning_result_bytes"]
        maximum_segments = limits["maximum_scout_plan_segments"]
        self.assertGreater(maximum_bytes, 0)
        self.assertGreater(maximum_segments, 0)
        self.assertIn("scout_4d_planning_result_v1", self.manifest["supported_features"])

        unknown = self.planning.ScoutPlanningResult.FromString(
            self._result().SerializeToString(deterministic=True) + b"\xa0\x06\x01"
        )
        with self.assertRaisesRegex(ValueError, "unknown"):
            validate_result(
                unknown, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=maximum_segments,
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=maximum_bytes,
            )

        oversized = self._result()
        source_segment = oversized.candidate.trajectory.segments[0]
        while len(oversized.candidate.trajectory.segments) <= maximum_segments:
            clone = oversized.candidate.trajectory.segments.add()
            clone.CopyFrom(source_segment)
        oversized.result_content_identity.sha256 = canonical_business_identity(
            oversized, "result_content_identity", exclude_header=True
        )
        with self.assertRaisesRegex(ValueError, "segments|RESOURCE_LIMIT"):
            validate_result(
                oversized, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=maximum_segments,
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=max(maximum_bytes, len(oversized.SerializeToString()) + 1),
            )

        tampered = self._result()
        tampered.candidate.trajectory.segments[0].position_control_points[0].x_m = 0.25
        tampered.result_content_identity.sha256 = canonical_business_identity(
            tampered, "result_content_identity", exclude_header=True
        )
        with self.assertRaisesRegex(ValueError, "trajectory content identity"):
            validate_result(
                tampered, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=maximum_segments,
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=maximum_bytes,
            )

    def test_normative_contract_hashing_and_adapter_rules_are_traceable(self) -> None:
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        hashing = HASHING_PATH.read_text(encoding="utf-8")
        for fragment in (
            "ScoutPlanningResult",
            "ScoutPlan",
            "mission_enu",
            "initial_yaw_rad",
            "yaw_offset_control_points_rad",
            "C2",
            "SurveyPlanEvidence",
            "ScoutPlanValidationReport",
            "INPUT_INVALID",
            "DEPENDENCY_STALE",
            "CAPABILITY_INFEASIBLE",
            "ENERGY_INSUFFICIENT",
            "COORDINATION_INFEASIBLE",
            "NO_SOLUTION",
            "SMOOTHING_FAILED",
            "SURVEY_INFEASIBLE",
            "VALIDATION_REJECTED",
            "VALIDATION_INCONCLUSIVE",
            "TIMEOUT",
            "CANCELLED",
            "NUMERICALLY_INVALID",
            "ROS 2",
            "C++",
            "NON_PRODUCTION",
        ):
            self.assertIn(fragment, contract)
        self.assertRegex(contract, r"MUST NOT[^\n]*(authorize|execution authority)")
        self.assertIn("ScoutTrajectory4d", hashing)
        self.assertIn("trajectory_content_identity", hashing)
        self.assertIn("dependencies_content_identity", hashing)
        self.assertIn("validation_report_content_identity", hashing)
        self.assertIn("plan_content_identity", hashing)
        self.assertIn("result_content_identity", hashing)

    def test_unknown_terminal_outcome_is_rejected_even_when_protobuf_parses_it(self) -> None:
        unknown = self._result()
        unknown.ClearField("candidate")
        unknown.outcome = 999
        unknown.diagnostics.add(
            numeric_code=131079,
            registry_id="underwater-system-codes",
            registry_version=1,
        )
        unknown.result_content_identity.sha256 = canonical_business_identity(
            unknown, "result_content_identity", exclude_header=True
        )
        limits = self.profile["interface_limits"]
        with self.assertRaisesRegex(ValueError, "unknown outcome"):
            validate_result(
                unknown, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=limits["maximum_scout_plan_segments"],
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=limits["maximum_scout_planning_result_bytes"],
            )

    def test_result_watermark_handles_duplicate_reorder_and_restart(self) -> None:
        session = b"0123456789abcdef"
        retired: set[bytes] = set()
        self.assertEqual(
            apply_result_watermark(
                current_session=session,
                current_delivery_sequence=41,
                current_result_sequence=33,
                current_identity=b"r" * 32,
                incoming_session=session,
                incoming_delivery_sequence=41,
                incoming_result_sequence=33,
                incoming_identity=b"r" * 32,
                retired_sessions=retired,
                recovery_boundary=False,
            ),
            "idempotent duplicate",
        )
        with self.assertRaisesRegex(ValueError, "delivery rollback"):
            apply_result_watermark(
                current_session=session,
                current_delivery_sequence=41,
                current_result_sequence=33,
                current_identity=b"r" * 32,
                incoming_session=session,
                incoming_delivery_sequence=40,
                incoming_result_sequence=34,
                incoming_identity=b"s" * 32,
                retired_sessions=retired,
                recovery_boundary=False,
            )
        with self.assertRaisesRegex(ValueError, "identity conflict"):
            apply_result_watermark(
                current_session=session,
                current_delivery_sequence=41,
                current_result_sequence=33,
                current_identity=b"r" * 32,
                incoming_session=session,
                incoming_delivery_sequence=41,
                incoming_result_sequence=33,
                incoming_identity=b"x" * 32,
                retired_sessions=retired,
                recovery_boundary=False,
            )

        new_session = b"fedcba9876543210"
        self.assertEqual(
            apply_result_watermark(
                current_session=session,
                current_delivery_sequence=41,
                current_result_sequence=33,
                current_identity=b"r" * 32,
                incoming_session=new_session,
                incoming_delivery_sequence=1,
                incoming_result_sequence=1,
                incoming_identity=b"n" * 32,
                retired_sessions=retired,
                recovery_boundary=True,
            ),
            "new session",
        )
        with self.assertRaisesRegex(ValueError, "retired"):
            apply_result_watermark(
                current_session=new_session,
                current_delivery_sequence=1,
                current_result_sequence=1,
                current_identity=b"n" * 32,
                incoming_session=session,
                incoming_delivery_sequence=42,
                incoming_result_sequence=34,
                incoming_identity=b"s" * 32,
                retired_sessions=retired,
                recovery_boundary=False,
            )

    def test_safe_report_requires_complete_finite_hard_gate_metrics(self) -> None:
        missing_metric = self._result()
        missing_metric.candidate.validation_report.ClearField(
            "minimum_collision_margin_m"
        )
        missing_metric.candidate.validation_report.validation_report_content_identity.sha256 = canonical_business_identity(
            missing_metric.candidate.validation_report,
            "validation_report_content_identity",
        )
        missing_metric.candidate.plan_content_identity.sha256 = canonical_business_identity(
            missing_metric.candidate, "plan_content_identity"
        )
        missing_metric.result_content_identity.sha256 = canonical_business_identity(
            missing_metric, "result_content_identity", exclude_header=True
        )
        limits = self.profile["interface_limits"]
        with self.assertRaisesRegex(ValueError, "SAFE.*metrics|non-finite"):
            validate_result(
                missing_metric, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=limits["maximum_scout_plan_segments"],
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=limits["maximum_scout_planning_result_bytes"],
            )

        nonfinite = self._result()
        nonfinite.candidate.validation_report.minimum_energy_margin_j = math.nan
        nonfinite.candidate.validation_report.validation_report_content_identity.sha256 = canonical_business_identity(
            nonfinite.candidate.validation_report,
            "validation_report_content_identity",
        )
        nonfinite.candidate.plan_content_identity.sha256 = canonical_business_identity(
            nonfinite.candidate, "plan_content_identity"
        )
        nonfinite.result_content_identity.sha256 = canonical_business_identity(
            nonfinite, "result_content_identity", exclude_header=True
        )
        with self.assertRaisesRegex(ValueError, "SAFE.*metrics|non-finite"):
            validate_result(
                nonfinite, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=limits["maximum_scout_plan_segments"],
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=limits["maximum_scout_planning_result_bytes"],
            )

    def test_failure_dependency_summary_identity_is_recomputed(self) -> None:
        failure = self._result()
        failure.ClearField("candidate")
        failure.outcome = self.codes.OUTCOME_TIMEOUT
        failure.diagnostics.add(
            numeric_code=131075,
            registry_id="underwater-system-codes",
            registry_version=1,
        )
        failure.dependencies.map_version += 1
        failure.result_content_identity.sha256 = canonical_business_identity(
            failure, "result_content_identity", exclude_header=True
        )
        limits = self.profile["interface_limits"]
        with self.assertRaisesRegex(ValueError, "failure dependency summary"):
            validate_result(
                failure, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=limits["maximum_scout_plan_segments"],
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=limits["maximum_scout_planning_result_bytes"],
            )

    def test_nested_nonfinite_and_non_nfc_business_values_fail_before_hash_install(self) -> None:
        limits = self.profile["interface_limits"]
        nonfinite = self._result()
        nonfinite.candidate.survey_evidence.conservative_predicted_coverage_ratio = math.nan
        nonfinite.candidate.survey_evidence.evidence_content_identity.sha256 = canonical_business_identity(
            nonfinite.candidate.survey_evidence, "evidence_content_identity"
        )
        nonfinite.candidate.validation_report.validated_survey_evidence_content_identity.CopyFrom(
            nonfinite.candidate.survey_evidence.evidence_content_identity
        )
        nonfinite.candidate.validation_report.validation_report_content_identity.sha256 = canonical_business_identity(
            nonfinite.candidate.validation_report, "validation_report_content_identity"
        )
        nonfinite.candidate.plan_content_identity.sha256 = canonical_business_identity(
            nonfinite.candidate, "plan_content_identity"
        )
        nonfinite.result_content_identity.sha256 = canonical_business_identity(
            nonfinite, "result_content_identity", exclude_header=True
        )
        with self.assertRaisesRegex(ValueError, "non-finite"):
            validate_result(
                nonfinite, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=limits["maximum_scout_plan_segments"],
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=limits["maximum_scout_planning_result_bytes"],
            )

        non_nfc = self._result()
        decomposed = "hybrid-ma\u0301p/scout"
        non_nfc.dependencies.map_id = decomposed
        non_nfc.candidate.dependencies.map_id = decomposed
        non_nfc.candidate.survey_evidence.baseline_map_id = decomposed
        non_nfc.dependencies.dependencies_content_identity.sha256 = canonical_business_identity(
            non_nfc.dependencies, "dependencies_content_identity"
        )
        non_nfc.candidate.dependencies.dependencies_content_identity.CopyFrom(
            non_nfc.dependencies.dependencies_content_identity
        )
        non_nfc.candidate.survey_evidence.evidence_content_identity.sha256 = canonical_business_identity(
            non_nfc.candidate.survey_evidence, "evidence_content_identity"
        )
        non_nfc.candidate.validation_report.validated_dependencies_content_identity.CopyFrom(
            non_nfc.candidate.dependencies.dependencies_content_identity
        )
        non_nfc.candidate.validation_report.validated_survey_evidence_content_identity.CopyFrom(
            non_nfc.candidate.survey_evidence.evidence_content_identity
        )
        non_nfc.candidate.validation_report.validation_report_content_identity.sha256 = canonical_business_identity(
            non_nfc.candidate.validation_report, "validation_report_content_identity"
        )
        non_nfc.candidate.plan_content_identity.sha256 = canonical_business_identity(
            non_nfc.candidate, "plan_content_identity"
        )
        non_nfc.result_content_identity.sha256 = canonical_business_identity(
            non_nfc, "result_content_identity", exclude_header=True
        )
        with self.assertRaisesRegex(ValueError, "non-NFC"):
            validate_result(
                non_nfc, self.common, self.codes, self.planning,
                manifest=self.manifest,
                accepted_manifest_identity=self.accepted_manifest_identity,
                maximum_segments=limits["maximum_scout_plan_segments"],
                maximum_diagnostics=limits["maximum_diagnostics"],
                maximum_bytes=limits["maximum_scout_planning_result_bytes"],
            )

    def test_early_input_failure_preserves_unavailable_dependency_facts(self) -> None:
        dependencies = self.planning.ScoutPlanningDependencies()
        dependencies.dependencies_content_identity.sha256 = canonical_business_identity(
            dependencies, "dependencies_content_identity"
        )
        failure = self.planning.ScoutPlanningResult(
            header=self._header(),
            result_sequence=33,
            outcome=self.codes.OUTCOME_INPUT_INVALID,
            evaluated_at_monotonic_ns=2_500_000_000,
            dependencies=dependencies,
            diagnostics=[
                self.common.CodeRef(
                    numeric_code=131079,
                    registry_id="underwater-system-codes",
                    registry_version=1,
                )
            ],
        )
        failure.result_content_identity.sha256 = canonical_business_identity(
            failure, "result_content_identity", exclude_header=True
        )
        limits = self.profile["interface_limits"]
        validate_result(
            failure, self.common, self.codes, self.planning,
            manifest=self.manifest,
            accepted_manifest_identity=self.accepted_manifest_identity,
            maximum_segments=limits["maximum_scout_plan_segments"],
            maximum_diagnostics=limits["maximum_diagnostics"],
            maximum_bytes=limits["maximum_scout_planning_result_bytes"],
        )


if __name__ == "__main__":
    unittest.main()
