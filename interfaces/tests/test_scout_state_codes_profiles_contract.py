"""Executable conformance checks for Scout state, code, and profile rules."""

from __future__ import annotations

import hashlib
import json
import pathlib
import unittest


SYSTEM_ROOT = pathlib.Path(__file__).resolve().parents[2]
INTERFACES = SYSTEM_ROOT / "interfaces"
PROTO_V1 = INTERFACES / "proto" / "underwater" / "contracts" / "v1"
STATE_REGISTRY_PATH = INTERFACES / "registry" / "scout-state-transitions-v1.json"
CODE_REGISTRY_PATH = INTERFACES / "registry" / "codes-v1.json"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"
CONTRACT_PATH = INTERFACES / "SCOUT_STATE_CODES_PROFILES.md"


def evaluate_transition(
    registry: dict[str, object],
    *,
    domain: str,
    previous_state: str,
    next_state: str,
    trigger: str,
) -> tuple[bool, str]:
    domain_rules = registry["domains"].get(domain)
    if domain_rules is None:
        return False, "AUDIT_ILLEGAL_TRANSITION_REJECTED"
    for transition in domain_rules["transitions"]:
        if (
            transition["from"] == previous_state
            and transition["to"] == next_state
            and transition["trigger"] == trigger
        ):
            return True, "AUDIT_STATE_TRANSITION"
    return False, "AUDIT_ILLEGAL_TRANSITION_REJECTED"


class ScoutStateCodesProfilesContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.states = json.loads(STATE_REGISTRY_PATH.read_text(encoding="utf-8"))
        cls.codes = json.loads(CODE_REGISTRY_PATH.read_text(encoding="utf-8"))
        cls.profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        cls.contract = CONTRACT_PATH.read_text(encoding="utf-8")

    def test_state_registry_covers_orthogonal_scout_domains_and_risk_actions(self) -> None:
        self.assertEqual(self.states["registry_id"], "scout-state-transitions/v1")
        self.assertEqual(
            set(self.states["domains"]),
            {
                "SCOUT_MISSION",
                "SCOUT_EXECUTION_AUTHORITY",
                "MAIN_SCOUT_COMMUNICATION",
                "SCOUT_FCU_COMMUNICATION",
            },
        )
        self.assertEqual(
            set(self.states["risk_action_rules"]),
            {"BRAKE", "HOLD", "RETREAT", "RETURN", "ASCEND"},
        )
        for rule in self.states["risk_action_rules"].values():
            self.assertTrue(rule["requires_revocation"])
            self.assertFalse(rule["permits_new_exploration"])
            self.assertIn(rule["terminal_transition"], {"REPLANNING", "FAILED"})

    def test_legal_transitions_are_exact_and_cross_domain_changes_are_rejected(self) -> None:
        accepted = (
            ("SCOUT_MISSION", "IDLE", "PLANNING", "MISSION_ACCEPTED"),
            (
                "SCOUT_EXECUTION_AUTHORITY",
                "NO_AUTHORIZATION",
                "AUTHORIZED",
                "SCOUT_BUNDLE_INSTALLED",
            ),
            (
                "MAIN_SCOUT_COMMUNICATION",
                "NORMAL",
                "DEGRADED",
                "LINK_DEGRADED",
            ),
            (
                "SCOUT_FCU_COMMUNICATION",
                "LOST",
                "RESYNCHRONIZING",
                "LINK_RESYNC_STARTED",
            ),
        )
        for domain, previous, next_state, trigger in accepted:
            self.assertEqual(
                evaluate_transition(
                    self.states,
                    domain=domain,
                    previous_state=previous,
                    next_state=next_state,
                    trigger=trigger,
                ),
                (True, "AUDIT_STATE_TRANSITION"),
            )

        rejected = (
            ("SCOUT_MISSION", "IDLE", "EXECUTING", "MISSION_ACCEPTED"),
            (
                "SCOUT_EXECUTION_AUTHORITY",
                "REVOKED",
                "AUTHORIZED",
                "LINK_RECOVERED",
            ),
            (
                "MAIN_SCOUT_COMMUNICATION",
                "LOST",
                "NORMAL",
                "LINK_RECOVERED",
            ),
            (
                "SCOUT_FCU_COMMUNICATION",
                "NORMAL",
                "AUTHORIZED",
                "SCOUT_BUNDLE_INSTALLED",
            ),
        )
        for domain, previous, next_state, trigger in rejected:
            self.assertEqual(
                evaluate_transition(
                    self.states,
                    domain=domain,
                    previous_state=previous,
                    next_state=next_state,
                    trigger=trigger,
                ),
                (False, "AUDIT_ILLEGAL_TRANSITION_REJECTED"),
            )

    def test_transition_triggers_are_stable_codes_and_every_result_is_audited(self) -> None:
        trigger_codes = {
            entry["name"]: entry["code"]
            for entry in self.codes["transition_trigger_codes"]
        }
        seen: set[tuple[str, str, str, str]] = set()
        for domain_name, domain in self.states["domains"].items():
            for transition in domain["transitions"]:
                key = (
                    domain_name,
                    transition["from"],
                    transition["to"],
                    transition["trigger"],
                )
                self.assertNotIn(key, seen)
                seen.add(key)
                self.assertEqual(
                    transition["trigger_code"], trigger_codes[transition["trigger"]]
                )
                self.assertEqual(transition["audit_event"], "STATE_TRANSITION")
                self.assertTrue(transition["required_evidence"])
        self.assertTrue(self.states["illegal_transition_rule"]["reject"])
        self.assertEqual(
            self.states["illegal_transition_rule"]["audit_event"],
            "ILLEGAL_TRANSITION_REJECTED",
        )

    def test_scout_codes_have_stable_numeric_identity_and_safety_semantics(self) -> None:
        expected_faults = {
            "SCOUT_FCU_HEARTBEAT_LOST",
            "SCOUT_FCU_COMMAND_STALE",
            "SCOUT_THRUSTER_HEALTH_INVALID",
            "SCOUT_ENERGY_RESERVE_INSUFFICIENT",
            "SCOUT_SENSOR_HEALTH_INVALID",
            "SCOUT_COORDINATION_INVALID",
            "SCOUT_CONFIGURATION_INCOMPATIBLE",
            "SCOUT_RISK_ACTION_FAILED",
            "SCOUT_MAP_DEPENDENCY_CHANGED",
        }
        faults = {
            entry["name"]: entry
            for entry in self.codes["fault_codes"]
            if entry["name"].startswith("SCOUT_")
        }
        self.assertEqual(set(faults), expected_faults)
        for fault in faults.values():
            self.assertGreater(fault["code"], 0)
            self.assertIn(fault["severity"], {"warning", "critical", "emergency"})
            self.assertTrue(fault["safety_effect"])
            self.assertIn(fault["latching"], {"latched", "self_clearing_condition"})
            self.assertTrue(fault["clear_authority"])
            self.assertTrue(fault["recovery_chain"])

        scout_diagnostics = [
            entry
            for entry in self.codes["diagnostic_codes"]
            if entry["name"].startswith("SCOUT_")
        ]
        self.assertGreaterEqual(len(scout_diagnostics), 7)
        for diagnostic in scout_diagnostics:
            self.assertIn(diagnostic["severity"], {"info", "warning", "critical"})
            self.assertTrue(diagnostic["safety_effect"])
            self.assertFalse(diagnostic["grants_authority"])

    def test_profile_atomically_references_all_scout_configuration_inputs(self) -> None:
        configuration = self.profile["scout_configuration"]
        self.assertFalse(configuration["production"])
        required_refs = {
            "timing_profile",
            "interface_limits",
            "code_registry",
            "state_transition_registry",
            "capability_profile",
            "energy_model",
            "planner_configuration",
            "safety_gate_configuration",
        }
        self.assertTrue(required_refs.issubset(configuration["references"]))
        self.assertGreaterEqual(len(configuration["sensor_geometry_profiles"]), 1)
        for reference in (
            list(configuration["references"].values())
            + configuration["sensor_geometry_profiles"]
        ):
            self.assertTrue(reference["id"])
            self.assertGreater(reference["version"], 0)
            self.assertEqual(len(reference["sha256"]), 64)
            self.assertNotEqual(reference["sha256"], "0" * 64)

    def test_loss_policies_are_channel_specific_fail_closed_and_need_fresh_sessions(self) -> None:
        policies = self.profile["scout_configuration"]["loss_policies"]
        self.assertEqual(set(policies), {"main_scout_coop", "scout_nuc_fcu"})
        self.assertNotEqual(policies["main_scout_coop"], policies["scout_nuc_fcu"])
        for policy in policies.values():
            self.assertFalse(policy["degraded"]["allow_new_authorization"])
            self.assertFalse(policy["lost"]["allow_new_authorization"])
            self.assertTrue(policy["lost"]["revoke_active_authorization"])
            self.assertIn(
                policy["lost"]["risk_action"],
                {"BRAKE", "HOLD", "RETREAT", "RETURN", "ASCEND"},
            )
            self.assertTrue(policy["recovery"]["requires_new_session"])
            self.assertTrue(policy["recovery"]["requires_fresh_evidence"])
            self.assertFalse(policy["recovery"]["restores_authorization"])

    def test_scout_timeout_and_resource_invariants_are_complete(self) -> None:
        timing = self.profile["timing"]
        self.assertLess(
            timing["scout_fcu_heartbeat_period_ns"],
            timing["scout_fcu_degraded_timeout_ns"],
        )
        self.assertLess(
            timing["scout_fcu_degraded_timeout_ns"],
            timing["scout_fcu_lost_timeout_ns"],
        )
        self.assertLessEqual(
            timing["scout_fcu_lost_timeout_ns"],
            timing["scout_execution_feedback_software_revoke_ns"],
        )
        self.assertLess(
            timing["scout_execution_feedback_software_revoke_ns"],
            timing["scout_fcu_command_watchdog_ns"],
        )
        self.assertLessEqual(
            timing["scout_fcu_command_watchdog_ns"],
            timing["scout_hard_safety_timeout_ns"],
        )
        limits = self.profile["interface_limits"]
        for name in (
            "maximum_state_transition_bytes",
            "maximum_scout_status_bytes",
            "maximum_fault_report_bytes",
            "maximum_diagnostic_event_bytes",
            "maximum_safety_audit_event_bytes",
            "maximum_scout_configuration_bytes",
            "maximum_loss_policy_bytes",
        ):
            self.assertGreater(limits[name], 0)

    def test_schema_exposes_typed_configuration_and_audit_rejection_evidence(self) -> None:
        profiles = (PROTO_V1 / "profiles.proto").read_text(encoding="utf-8")
        diagnostics = (PROTO_V1 / "diagnostics.proto").read_text(encoding="utf-8")
        state = (PROTO_V1 / "state.proto").read_text(encoding="utf-8")
        for declaration in (
            "message ScoutConfigurationProfile",
            "message ScoutCommunicationLossPolicy",
            "message ScoutRiskActionRule",
        ):
            self.assertIn(declaration, profiles)
        self.assertIn("CodeRef trigger", state)
        for field in (
            "CodeRef outcome",
            "CodeRef fault",
            "repeated CodeRef diagnostics",
            "CodeRef transition_trigger",
            "rejected_enum_numeric_value",
        ):
            self.assertIn(field, diagnostics)

    def test_unknown_or_incompatible_inputs_have_deterministic_audit_results(self) -> None:
        rules = self.states["rejection_rules"]
        self.assertEqual(
            set(rules),
            {
                "UNKNOWN_SAFETY_ENUM",
                "ILLEGAL_STATE_TRANSITION",
                "MISSING_PRODUCTION_PARAMETER",
                "INCOMPATIBLE_PROFILE",
            },
        )
        for rule in rules.values():
            self.assertTrue(rule["reject"])
            self.assertTrue(rule["outcome"])
            self.assertTrue(rule["diagnostic"])
            self.assertTrue(rule["audit_event"])

    def test_manifest_hashes_and_normative_recovery_rules_are_exact(self) -> None:
        feature = "scout_state_codes_profiles_v1"
        self.assertIn(feature, self.manifest["supported_features"])
        self.assertEqual(
            hashlib.sha256(STATE_REGISTRY_PATH.read_bytes()).hexdigest(),
            self.manifest["scout_state_transition_registry"]["sha256"],
        )
        for phrase in (
            "正交",
            "unknown safety-critical enum",
            "illegal transition",
            "missing production parameter",
            "incompatible profile",
            "条件已消失",
            "新会话",
            "重新规划",
            "新授权执行包",
        ):
            self.assertIn(phrase, self.contract)


if __name__ == "__main__":
    unittest.main()
