"""Public-contract tests for the scout survey-mission lifecycle."""

from __future__ import annotations

import importlib
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest


INTERFACES = pathlib.Path(__file__).resolve().parents[1]
PROTO_ROOT = INTERFACES / "proto"
PROTO_V1 = PROTO_ROOT / "underwater" / "contracts" / "v1"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
LIFECYCLE_PATH = INTERFACES / "SCOUT_MISSION_LIFECYCLE.md"
HASHING_PATH = INTERFACES / "HASHING.md"
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"


class ScoutMissionLifecycleContract(unittest.TestCase):
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
        cls.cooperation = importlib.import_module(
            "underwater.contracts.v1.cooperation_pb2"
        )
        cls.common = importlib.import_module("underwater.contracts.v1.common_pb2")

    @classmethod
    def tearDownClass(cls) -> None:
        sys.path.remove(cls.generated.name)
        cls.generated.cleanup()

    def test_versioned_scout_mission_round_trip_preserves_contract(self) -> None:
        mission = self.cooperation.ScoutMission(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="laying-mission-authority",
                producer_session_id=b"0123456789abcdef",
                stream_id=self.common.STREAM_SCOUT_MISSION,
                sequence=41,
                source_clock_domain_id="main-nuc/boot-7",
                generated_at_monotonic_ns=1_000_000_000,
            ),
            mission_id=73,
            required_region=self.cooperation.Region3dEnu(
                xyz_m=[0.0, 1.0, -4.0, 3.0, 5.0, -2.0],
                frame_id="mission_enu",
            ),
            allowed_scout_region=self.cooperation.Region3dEnu(
                xyz_m=[-1.0, 0.0, -6.0, 5.0, 7.0, 0.0],
                frame_id="mission_enu",
            ),
            required_coverage_ratio=0.95,
            required_resolution_m=0.1,
            maximum_evidence_age_ns=2_000_000_000,
            business_deadline_monotonic_ns=9_000_000_000,
            urgency=self.cooperation.SURVEY_URGENCY_BLOCKING,
            minimum_separation_m=2.5,
            maximum_communication_distance_m=80.0,
            coordination_version=11,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
        )

        decoded = self.cooperation.ScoutMission.FromString(
            mission.SerializeToString(deterministic=True)
        )

        self.assertEqual(decoded, mission)
        self.assertEqual(decoded.mission_version, 4)
        self.assertEqual(decoded.mission_content_identity.sha256, b"m" * 32)
        self.assertEqual(decoded.header.source_clock_domain_id, "main-nuc/boot-7")
        self.assertEqual(decoded.business_deadline_monotonic_ns, 9_000_000_000)

    def test_admission_decision_binds_exact_mission_and_local_window(self) -> None:
        decision = self.cooperation.ScoutMissionDecision(
            header=self.common.MessageHeader(
                schema_major=1,
                schema_minor=0,
                producer_id="scout-mission-authority",
                producer_session_id=b"fedcba9876543210",
                stream_id=self.common.STREAM_SCOUT_MISSION_DECISION,
                sequence=9,
                source_clock_domain_id="scout-nuc/boot-3",
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            disposition=self.cooperation.SCOUT_MISSION_DECISION_ACCEPTED,
            outcome=self.common.CodeRef(
                numeric_code=2,
                registry_id="underwater-system-codes",
                registry_version=1,
            ),
            received_at_monotonic_ns=3_000_000_000,
            admission_valid_until_monotonic_ns=3_500_000_000,
            accepted_coordination_version=11,
            decision_content_identity=self.common.ContentIdentity(sha256=b"d" * 32),
        )

        decoded = self.cooperation.ScoutMissionDecision.FromString(
            decision.SerializeToString(deterministic=True)
        )

        self.assertEqual(decoded, decision)
        self.assertEqual(decoded.header.source_clock_domain_id, "scout-nuc/boot-3")
        self.assertLess(
            decoded.received_at_monotonic_ns,
            decoded.admission_valid_until_monotonic_ns,
        )

    def test_cancellation_and_ack_bind_the_exact_mission(self) -> None:
        cancellation = self.cooperation.ScoutMissionCancellation(
            header=self.common.MessageHeader(
                stream_id=self.common.STREAM_SCOUT_MISSION_CANCELLATION,
                source_clock_domain_id="main-nuc/boot-7",
                sequence=3,
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            cancellation_version=2,
            reason=self.common.CodeRef(
                numeric_code=22,
                registry_id="underwater-system-codes",
                registry_version=1,
            ),
            cancellation_content_identity=self.common.ContentIdentity(sha256=b"c" * 32),
        )
        acknowledgement = self.cooperation.ScoutMissionCancellationAck(
            header=self.common.MessageHeader(
                stream_id=self.common.STREAM_SCOUT_MISSION_CANCELLATION_ACK,
                source_clock_domain_id="scout-nuc/boot-3",
                sequence=3,
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            cancellation_version=2,
            cancellation_content_identity=self.common.ContentIdentity(sha256=b"c" * 32),
            disposition=self.cooperation.SCOUT_MISSION_CANCELLATION_APPLIED,
            outcome=self.common.CodeRef(
                numeric_code=22,
                registry_id="underwater-system-codes",
                registry_version=1,
            ),
            ack_content_identity=self.common.ContentIdentity(sha256=b"a" * 32),
        )

        decoded_cancellation = self.cooperation.ScoutMissionCancellation.FromString(
            cancellation.SerializeToString(deterministic=True)
        )
        decoded_ack = self.cooperation.ScoutMissionCancellationAck.FromString(
            acknowledgement.SerializeToString(deterministic=True)
        )

        self.assertEqual(decoded_cancellation, cancellation)
        self.assertEqual(decoded_ack, acknowledgement)
        self.assertEqual(
            decoded_ack.cancellation_content_identity,
            decoded_cancellation.cancellation_content_identity,
        )

    def test_plan_prediction_and_completion_evidence_are_not_interchangeable(self) -> None:
        plan_evidence = self.cooperation.SurveyPlanEvidence(
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            baseline_map_id="local-map",
            baseline_map_version=12,
            baseline_map_content_identity=self.common.ContentIdentity(sha256=b"b" * 32),
            conservative_predicted_coverage_ratio=0.96,
            predicted_resolution_m=0.08,
            predicted_covered_region=self.cooperation.Region3dEnu(
                xyz_m=[0.0, 1.0, -4.0, 3.0, 5.0, -2.0],
                frame_id="mission_enu",
            ),
            evidence_content_identity=self.common.ContentIdentity(sha256=b"p" * 32),
        )
        completion = self.cooperation.SurveyCompletionEvidence(
            header=self.common.MessageHeader(
                stream_id=self.common.STREAM_SURVEY_COMPLETION_EVIDENCE,
                source_clock_domain_id="scout-nuc/boot-3",
                sequence=5,
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            baseline_map_id="local-map",
            baseline_map_version=12,
            baseline_map_content_identity=self.common.ContentIdentity(sha256=b"b" * 32),
            resulting_map_id="local-map",
            resulting_map_version=13,
            resulting_map_content_identity=self.common.ContentIdentity(sha256=b"n" * 32),
            observation_ids=[b"obs-100", b"obs-101"],
            observed_covered_region=self.cooperation.Region3dEnu(
                xyz_m=[0.0, 1.0, -4.0, 3.0, 5.0, -2.0],
                frame_id="mission_enu",
            ),
            achieved_coverage_ratio=0.97,
            achieved_resolution_m=0.09,
            oldest_contributing_observation_time=self.common.SynchronizedObservationTime(
                utc_time_ns=1_800_000_000_000_000_000,
                status=self.common.TIME_SYNC_SYNCHRONIZED,
                uncertainty_ns=2_000_000,
            ),
            completion_content_identity=self.common.ContentIdentity(sha256=b"e" * 32),
        )

        decoded_plan = self.cooperation.SurveyPlanEvidence.FromString(
            plan_evidence.SerializeToString(deterministic=True)
        )
        decoded_completion = self.cooperation.SurveyCompletionEvidence.FromString(
            completion.SerializeToString(deterministic=True)
        )

        self.assertEqual(decoded_plan, plan_evidence)
        self.assertEqual(decoded_completion, completion)
        self.assertEqual(
            decoded_plan.baseline_map_content_identity,
            decoded_completion.baseline_map_content_identity,
        )
        self.assertNotIn("observation_ids", decoded_plan.DESCRIPTOR.fields_by_name)
        self.assertGreater(
            decoded_completion.resulting_map_version,
            decoded_completion.baseline_map_version,
        )
        self.assertEqual(list(decoded_completion.observation_ids), [b"obs-100", b"obs-101"])

    def test_completion_acknowledgement_binds_exact_evidence(self) -> None:
        acknowledgement = self.cooperation.SurveyCompletionAck(
            header=self.common.MessageHeader(
                stream_id=self.common.STREAM_SURVEY_COMPLETION_ACK,
                source_clock_domain_id="main-nuc/boot-7",
                sequence=8,
            ),
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            completion_content_identity=self.common.ContentIdentity(sha256=b"e" * 32),
            disposition=self.cooperation.SURVEY_COMPLETION_ACCEPTED,
            outcome=self.common.CodeRef(
                numeric_code=1,
                registry_id="underwater-system-codes",
                registry_version=1,
            ),
            ack_content_identity=self.common.ContentIdentity(sha256=b"k" * 32),
        )

        decoded = self.cooperation.SurveyCompletionAck.FromString(
            acknowledgement.SerializeToString(deterministic=True)
        )

        self.assertEqual(decoded, acknowledgement)
        self.assertEqual(decoded.completion_content_identity.sha256, b"e" * 32)

    def test_local_admission_window_is_bounded_before_link_loss(self) -> None:
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        timing = profile["timing"]

        self.assertGreater(timing["scout_mission_admission_window_ns"], 0)
        self.assertLess(
            timing["scout_mission_decision_timeout_ns"],
            timing["scout_mission_admission_window_ns"],
        )
        self.assertLess(
            timing["scout_mission_admission_window_ns"],
            timing["coop_lost_timeout_ns"],
        )
        profiles_schema = (PROTO_V1 / "profiles.proto").read_text(encoding="utf-8")
        self.assertIn("scout_mission_decision_timeout_ns", profiles_schema)
        self.assertIn("scout_mission_admission_window_ns", profiles_schema)

    def test_region_wire_encoding_is_an_unambiguous_enu_aabb(self) -> None:
        schema = (PROTO_V1 / "cooperation.proto").read_text(encoding="utf-8")

        self.assertIn(
            "Exactly [min_x, min_y, min_z, max_x, max_y, max_z]",
            schema,
        )
        self.assertIn("min values MUST be strictly less than max values", schema)

    def test_lifecycle_contract_declares_deterministic_fail_closed_results(self) -> None:
        lifecycle = LIFECYCLE_PATH.read_text(encoding="utf-8")
        required_rules = (
            "same sequence + same content identity | idempotent duplicate",
            "same sequence + different content identity | `INPUT_INVALID`",
            "lower sequence in the same producer session | `SEQUENCE_REJECTED`",
            "mission version rollback | `VERSION_INCOMPATIBLE`",
            "same mission version + different content identity | `INPUT_INVALID`",
            "local admission window expired | `DEPENDENCY_STALE`",
            "unknown safety-critical enum | `INPUT_INVALID`",
            "cancellation version rollback | `SEQUENCE_REJECTED`",
            "completion without observation IDs | `INPUT_INVALID`",
            "resulting map version not newer than baseline | `INPUT_INVALID`",
        )
        for rule in required_rules:
            self.assertIn(rule, lifecycle)

    def test_unknown_urgency_survives_wire_parse_and_is_explicitly_invalid(self) -> None:
        mission = self.cooperation.ScoutMission(urgency=99)
        decoded = self.cooperation.ScoutMission.FromString(mission.SerializeToString())
        known_values = {
            value.number
            for value in self.cooperation.SurveyUrgency.DESCRIPTOR.values
            if value.number != 0
        }

        self.assertEqual(decoded.urgency, 99)
        self.assertNotIn(decoded.urgency, known_values)
        lifecycle = LIFECYCLE_PATH.read_text(encoding="utf-8")
        self.assertIn("unknown safety-critical enum | `INPUT_INVALID`", lifecycle)

    def test_adapter_requirements_preserve_public_semantics(self) -> None:
        lifecycle = LIFECYCLE_PATH.read_text(encoding="utf-8")
        for rule in (
            "C++ <-> Protobuf",
            "ROS 2 <-> Protobuf",
            "preserve optional presence",
            "MUST NOT synthesize or replace",
            "field-by-field bidirectional tests",
        ):
            self.assertIn(rule, lifecycle)

    def test_scout_status_binds_the_exact_mission_version(self) -> None:
        status = self.cooperation.ScoutStatus(
            mission_id=73,
            mission_version=4,
            mission_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
        )

        decoded = self.cooperation.ScoutStatus.FromString(status.SerializeToString())

        self.assertEqual(decoded.mission_id, 73)
        self.assertEqual(decoded.mission_version, 4)
        self.assertEqual(decoded.mission_content_identity.sha256, b"m" * 32)

    def test_hashing_rules_cover_every_lifecycle_identity(self) -> None:
        hashing = HASHING_PATH.read_text(encoding="utf-8")
        for rule in (
            "`ScoutMission`: clear `mission_content_identity`",
            "`ScoutMissionDecision`: clear `decision_content_identity`",
            "`ScoutMissionCancellation`: clear `cancellation_content_identity`",
            "`ScoutMissionCancellationAck`: clear `ack_content_identity`",
            "`SurveyPlanEvidence`: clear `evidence_content_identity`",
            "`SurveyCompletionEvidence`: clear `completion_content_identity`",
            "`SurveyCompletionAck`: clear `ack_content_identity`",
            "exclude the top-level `MessageHeader` from the business content hash",
            "observation IDs MUST be unique and lexicographically ascending",
        ):
            self.assertIn(rule, hashing)

    def test_manifest_gates_scout_mission_lifecycle_as_exact_feature(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))

        self.assertEqual(manifest["schema_major"], 1)
        self.assertEqual(manifest["schema_minor"], 0)
        self.assertIn("scout_mission_lifecycle_v1", manifest["supported_features"])
        self.assertEqual(manifest["approved_mixed_versions"], [])


if __name__ == "__main__":
    unittest.main()
