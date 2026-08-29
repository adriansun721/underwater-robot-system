"""Public-contract tests for independent main and Scout execution authority domains."""

from __future__ import annotations

import pathlib
import re
import json
import unittest


SYSTEM_ROOT = pathlib.Path(__file__).resolve().parents[2]
INTERFACES = SYSTEM_ROOT / "interfaces"
PROTO_V1 = INTERFACES / "proto" / "underwater" / "contracts" / "v1"


class ExecutionAuthorityDomainContractTest(unittest.TestCase):
    def test_authority_domains_are_strongly_typed_and_disjoint(self) -> None:
        execution = (PROTO_V1 / "execution.proto").read_text(encoding="utf-8")
        state = (PROTO_V1 / "state.proto").read_text(encoding="utf-8")
        context = (SYSTEM_ROOT / "CONTEXT.md").read_text(encoding="utf-8")
        adr = (SYSTEM_ROOT / "docs" / "adr" / "0003-single-software-authority-independent-estop.md").read_text(
            encoding="utf-8"
        )

        self.assertRegex(execution, r"message\s+MainLayingExecutionAuthorityDomain\s*\{")
        self.assertRegex(execution, r"message\s+ScoutMotionExecutionAuthorityDomain\s*\{")
        self.assertRegex(
            execution,
            r"MainLayingExecutionAuthorityDomain\s+domain\s*=\s*12\s*;",
        )
        self.assertRegex(state, r"STATE_DOMAIN_MAIN_EXECUTION_AUTHORITY\s*=\s*1\s*;")
        self.assertRegex(state, r"STATE_DOMAIN_SCOUT_EXECUTION_AUTHORITY\s*=\s*7\s*;")
        self.assertNotRegex(context, r"(?m)^.*(message|field|protobuf|sequence).*ExecutionAuthority.*$")
        self.assertIn("每个物理执行授权域", context)
        self.assertIn("MUST NOT sign, renew, revoke, or share watermarks across domains", adr)

    def test_publishers_states_clocks_and_mixed_versions_fail_closed(self) -> None:
        state = (PROTO_V1 / "state.proto").read_text(encoding="utf-8")
        integration = (SYSTEM_ROOT / "docs" / "system-integration-contract.md").read_text(
            encoding="utf-8"
        )
        transitions = (SYSTEM_ROOT / "docs" / "state-machine-contract.md").read_text(
            encoding="utf-8"
        )
        manifest = json.loads(
            (INTERFACES / "compatibility" / "contract-manifest-v1.json").read_text(
                encoding="utf-8"
            )
        )

        self.assertRegex(
            state,
            r"ExecutionAuthorityState\s+scout_authority_state\s*=\s*10\s*;",
        )
        self.assertIn("Main laying ExecutionAuthority", integration)
        self.assertIn("Scout motion ExecutionAuthority", integration)
        self.assertRegex(integration, r"main NUC local\s+monotonic clock")
        self.assertRegex(integration, r"Scout NUC local\s+monotonic clock")
        self.assertIn("must be re-issued", integration)
        self.assertIn("## MainLayingExecutionAuthority", transitions)
        self.assertIn("## ScoutMotionExecutionAuthority", transitions)
        self.assertIn("No transition in either domain changes the other domain", transitions)
        self.assertIn(
            "independent_execution_authority_domains_v1",
            manifest["supported_features"],
        )
        self.assertEqual(manifest["approved_mixed_versions"], [])


if __name__ == "__main__":
    unittest.main()
