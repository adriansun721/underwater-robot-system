"""Transport-independent static conformance checks for the v1 contracts."""

from __future__ import annotations

import json
import hashlib
import pathlib
import re
import subprocess
import tempfile
import unittest


INTERFACES = pathlib.Path(__file__).resolve().parents[1]
PROTO_ROOT = INTERFACES / "proto"
PROTO_V1 = PROTO_ROOT / "underwater" / "contracts" / "v1"
REGISTRY_PATH = INTERFACES / "registry" / "codes-v1.json"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
DBC_PATH = INTERFACES / "can" / "main_robot_can_v1.dbc"
CAN_SUPPLEMENT_PATH = INTERFACES / "can" / "main_robot_can_v1.md"
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"


def enum_values(proto_text: str, enum_name: str, prefix: str) -> dict[str, int]:
    match = re.search(rf"enum\s+{enum_name}\s*\{{(.*?)\}}", proto_text, re.S)
    if not match:
        raise AssertionError(f"missing enum {enum_name}")
    result: dict[str, int] = {}
    for name, number in re.findall(r"^\s*([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*;", match.group(1), re.M):
        if number == "0":
            continue
        if not name.startswith(prefix):
            raise AssertionError(f"{name} does not start with {prefix}")
        result[name[len(prefix) :]] = int(number)
    return result


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class ContractConformance(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.registry = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
        cls.profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        cls.codes_proto = (PROTO_V1 / "codes.proto").read_text(encoding="utf-8")
        cls.dbc = DBC_PATH.read_text(encoding="utf-8")
        cls.manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))

    def test_all_protobuf_schemas_compile(self) -> None:
        proto_files = sorted(str(path) for path in PROTO_V1.glob("*.proto"))
        self.assertGreaterEqual(len(proto_files), 7)
        with tempfile.TemporaryDirectory() as directory:
            descriptor = pathlib.Path(directory) / "contracts.pb"
            subprocess.run(
                [
                    "protoc",
                    f"--proto_path={PROTO_ROOT}",
                    "--include_imports",
                    f"--descriptor_set_out={descriptor}",
                    *proto_files,
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertGreater(descriptor.stat().st_size, 0)

    def test_contract_manifest_hashes_match_canonical_artifacts(self) -> None:
        proto_files = sorted(str(path) for path in PROTO_V1.glob("*.proto"))
        with tempfile.TemporaryDirectory() as directory:
            descriptor = pathlib.Path(directory) / "contracts.pb"
            subprocess.run(
                [
                    "protoc",
                    f"--proto_path={PROTO_ROOT}",
                    "--include_imports",
                    f"--descriptor_set_out={descriptor}",
                    *proto_files,
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                hashlib.sha256(descriptor.read_bytes()).hexdigest(),
                self.manifest["protobuf_descriptor_sha256"],
            )
        self.assertEqual(
            hashlib.sha256(REGISTRY_PATH.read_bytes()).hexdigest(),
            self.manifest["code_registry"]["sha256"],
        )
        self.assertEqual(
            hashlib.sha256(PROFILE_PATH.read_bytes()).hexdigest(),
            self.manifest["integration_profile"]["sha256"],
        )
        self.assertEqual(
            hashlib.sha256(DBC_PATH.read_bytes()).hexdigest(),
            self.manifest["can_protocol"]["dbc_sha256"],
        )
        self.assertEqual(
            hashlib.sha256(CAN_SUPPLEMENT_PATH.read_bytes()).hexdigest(),
            self.manifest["can_protocol"]["supplement_sha256"],
        )
        self.assertEqual(
            hashlib.sha256(DBC_PATH.read_bytes() + CAN_SUPPLEMENT_PATH.read_bytes()).hexdigest(),
            self.manifest["can_protocol"]["combined_sha256"],
        )

    def test_registry_matches_protobuf_numeric_codes(self) -> None:
        expected = {
            "outcome_codes": enum_values(self.codes_proto, "OutcomeCode", "OUTCOME_"),
            "fault_codes": enum_values(self.codes_proto, "FaultCode", "FAULT_"),
            "diagnostic_codes": enum_values(self.codes_proto, "DiagnosticCode", "DIAGNOSTIC_"),
        }
        for group, proto_values in expected.items():
            registry_values = {entry["name"]: entry["code"] for entry in self.registry[group]}
            self.assertEqual(proto_values, registry_values, group)

    def test_fault_codes_have_local_safety_semantics(self) -> None:
        required = {"code", "name", "domain", "severity", "safety_effect", "latching", "clear_authority"}
        codes: set[int] = set()
        for entry in self.registry["fault_codes"]:
            self.assertTrue(required.issubset(entry), entry)
            self.assertNotIn(entry["code"], codes)
            codes.add(entry["code"])

    def test_public_schema_uses_enu_yaw_not_heading(self) -> None:
        combined = "\n".join(path.read_text(encoding="utf-8") for path in PROTO_V1.glob("*.proto"))
        self.assertNotRegex(combined.lower(), r"\bheading\b")
        self.assertIn("yaw_rad", combined)
        self.assertIn("mission_enu", combined)
        self.assertIn("base_link", combined)

    def test_integration_profile_is_non_production_and_bounded(self) -> None:
        self.assertEqual(self.profile["profile_id"], "integration/v1")
        self.assertFalse(self.profile["production"])
        for value in self.profile["interface_limits"].values():
            self.assertIsInstance(value, int)
            self.assertGreater(value, 0)

    def test_timeout_ordering_invariant(self) -> None:
        timing = self.profile["timing"]
        self.assertLess(timing["execution_feedback_publish_period_ns"], timing["execution_feedback_stale_warning_ns"])
        self.assertLess(timing["execution_feedback_stale_warning_ns"], timing["execution_feedback_software_revoke_ns"])
        self.assertLess(timing["execution_feedback_software_revoke_ns"], timing["can_command_watchdog_ns"])
        self.assertLessEqual(timing["can_command_watchdog_ns"], timing["hard_safety_timeout_ns"])
        self.assertLess(timing["lease_renewal_margin_ns"], timing["lease_duration_ns"])
        self.assertLess(timing["authorized_bundle_ack_timeout_ns"], timing["lease_duration_ns"])

    def test_can_protocol_contains_required_safety_frames(self) -> None:
        required = {
            "SYS_HEARTBEAT": 1,
            "SYS_ESTOP": 2,
            "SYS_SESSION": 3,
            "CMD_TRACK_CTRL": 256,
            "CMD_CABLE_CTRL": 272,
            "FB_TRACK_STATUS": 512,
            "FB_CABLE_STATUS": 528,
            "CFG_PROFILE_PREPARE": 768,
            "CFG_PROFILE_HASH_CHUNK": 769,
            "CFG_PROFILE_ACTIVATE": 770,
            "FB_PROFILE_STATUS": 771,
        }
        messages = {name: int(identifier) for identifier, name in re.findall(r"BO_\s+(\d+)\s+(\w+):", self.dbc)}
        self.assertEqual({name: messages[name] for name in required}, required)

    def test_can_signals_fit_without_overlap_and_protected_frames_have_crc(self) -> None:
        starts = list(re.finditer(r"^BO_\s+(\d+)\s+(\w+):\s+(\d+)\s+\w+", self.dbc, re.M))
        messages: dict[str, set[str]] = {}
        for index, match in enumerate(starts):
            end = starts[index + 1].start() if index + 1 < len(starts) else len(self.dbc)
            block = self.dbc[match.end() : end]
            payload_bits = int(match.group(3)) * 8
            occupied: set[int] = set()
            signal_names: set[str] = set()
            for signal, start, length in re.findall(r"^\s*SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@", block, re.M):
                first, width = int(start), int(length)
                self.assertLessEqual(first + width, payload_bits, (match.group(2), signal))
                bits = set(range(first, first + width))
                self.assertFalse(occupied.intersection(bits), (match.group(2), signal))
                occupied.update(bits)
                signal_names.add(signal)
            messages[match.group(2)] = signal_names
        for name in (
            "SYS_HEARTBEAT",
            "SYS_ESTOP",
            "SYS_SESSION",
            "CMD_TRACK_CTRL",
            "CMD_CABLE_CTRL",
            "CMD_CABLE_AUX",
            "CFG_PROFILE_PREPARE",
            "CFG_PROFILE_HASH_CHUNK",
            "CFG_PROFILE_ACTIVATE",
            "FB_PROFILE_STATUS",
        ):
            self.assertIn("Crc16", messages[name], name)

    def test_can_fault_mapping_is_complete_and_unambiguous(self) -> None:
        fault_codes = {entry["code"] for entry in self.registry["fault_codes"]}
        occupied: set[tuple[str, str, int]] = set()
        for mapping in self.registry["can_fault_mappings"]:
            key = (mapping["message"], mapping["signal"], mapping["bit"])
            self.assertNotIn(key, occupied)
            occupied.add(key)
            self.assertIn(mapping["fault_code"], fault_codes)
            self.assertRegex(self.dbc, rf"BO_\s+\d+\s+{mapping['message']}:")
            self.assertRegex(self.dbc, rf"SG_\s+{mapping['signal']}\s*:")
            self.assertGreaterEqual(mapping["bit"], 0)
            self.assertLess(mapping["bit"], 8)

    def test_can_crc_golden_vector(self) -> None:
        # CAN ID 0x100 LE + payload[0:6]: +1.000 m/s left/right, mode 1, seq 7.
        protected = bytes([0x00, 0x01, 0xE8, 0x03, 0xE8, 0x03, 0x01, 0x07])
        self.assertEqual(crc16_ccitt_false(protected), 0xFA58)

    def test_atomic_bundle_and_feedback_triplet_are_explicit(self) -> None:
        execution = (PROTO_V1 / "execution.proto").read_text(encoding="utf-8")
        for field in (
            "ImmutablePlan plan",
            "ExecutionLease lease",
            "execution_epoch_monotonic_ns",
            "bundle_content_identity",
            "ControlVector profile_target",
            "AppliedControlVector applied_target",
            "MeasuredExecutionState measured_state",
            "LocalSafetyOverride safety_override",
        ):
            self.assertIn(field, execution)

    def test_zero_valid_safety_scalars_have_explicit_presence(self) -> None:
        common = (PROTO_V1 / "common.proto").read_text(encoding="utf-8")
        planning = (PROTO_V1 / "planning.proto").read_text(encoding="utf-8")
        execution = (PROTO_V1 / "execution.proto").read_text(encoding="utf-8")
        for field in (
            "optional double x_m",
            "optional double y_m",
            "optional double yaw_rad",
            "optional int64 generated_at_monotonic_ns",
        ):
            self.assertIn(field, common)
        for field in (
            "optional uint64 time_from_execution_epoch_ns",
            "optional double ground_speed_mps",
            "optional double payout_speed_mps",
            "optional double tension_setpoint_n",
        ):
            self.assertIn(field, planning)
        for field in (
            "optional bool limit_applied",
            "optional uint64 profile_time_ns",
            "optional double profile_arc_length_m",
            "optional uint32 can_profile_sequence",
        ):
            self.assertIn(field, execution)

    def test_safety_contract_documents_fail_closed_rules(self) -> None:
        system_doc = (INTERFACES.parent / "docs" / "system-integration-contract.md").read_text(encoding="utf-8")
        for phrase in (
            "MUST NOT cache",
            "unknown\nsafety-critical enum MUST reject",
            "MUST NOT reset or shift",
            "MUST NOT truncate or",
            "must not block safety channels",
        ):
            self.assertIn(phrase, system_doc)

    def test_content_identity_canonicalization_is_explicit(self) -> None:
        hashing = (INTERFACES / "HASHING.md").read_text(encoding="utf-8")
        for phrase in (
            "reject unknown fields",
            "negative zero to positive zero",
            "Unicode NFC",
            "ascending field-number order",
            "Map fields are prohibited",
            "constant time",
        ):
            self.assertIn(phrase, hashing)


if __name__ == "__main__":
    unittest.main()
