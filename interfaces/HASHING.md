# v1 content identity rules

`ContentIdentity.sha256` is exactly 32 bytes. It proves immutable content
identity/integrity inside the trusted v1 control domain; it is not source
authentication.

## Protobuf objects

Before hashing a Plan, ExecutionProfile, lease, Bundle, or controlled-stop
profile, the producer MUST:

1. validate all required presence, enums, finite values, ranges, units,
   ordering, and InterfaceLimits;
2. reject unknown fields rather than include or discard them silently;
3. normalize every ENU yaw to `[-pi, pi)`;
4. normalize floating-point negative zero to positive zero and reject all
   NaN/infinity representations;
5. normalize strings to Unicode NFC and reject invalid UTF-8;
6. clear only the identity field of the object currently being hashed while
   retaining the already-validated identities of nested immutable objects;
7. serialize known fields in ascending field-number order, preserve repeated
   element order, preserve optional presence, and use the shortest legal
   Protobuf scalar encoding;
8. compute SHA-256 over those bytes.

A bootstrap `ContractManifest` follows the same rules with
`manifest_content_identity` cleared. Its `MessageHeader.manifest` is absent to
avoid a self-reference cycle. After a Manifest is accepted, every other
top-level public message must reference that exact identity.

Map fields are prohibited inside hashed safety objects in v1 because ordering
would otherwise require another canonicalization rule. The current hashed
messages contain no maps.

For `AuthorizedExecutionBundle`, `bundle_content_identity` is cleared while
the strong main-laying domain marker, Plan, ExecutionProfile, lease, Profile,
and Manifest identities remain.
An at-least-once retransmission reuses the exact same header, sequence, event
identity, and Bundle bytes.

Scout mission lifecycle objects use the same algorithm with the following
single self-identity field cleared and every referenced identity retained:

- `ScoutMission`: clear `mission_content_identity`;
- `ScoutMissionDecision`: clear `decision_content_identity`;
- `ScoutMissionCancellation`: clear `cancellation_content_identity`;
- `ScoutMissionCancellationAck`: clear `ack_content_identity`;
- `SurveyPlanEvidence`: clear `evidence_content_identity`;
- `SurveyCompletionEvidence`: clear `completion_content_identity`;
- `SurveyCompletionAck`: clear `ack_content_identity`.

For these lifecycle business identities, exclude the top-level `MessageHeader` from the business content hash.
The header is the delivery envelope and changes across producer sessions; it
cannot change the identity of an otherwise identical mission, cancellation,
decision, completion, or acknowledgement. `SurveyPlanEvidence` has no header.
Within one producer session, at-least-once retransmission still reuses the
exact complete bytes, including the header.

Before hashing `SurveyCompletionEvidence`, observation IDs MUST be unique and lexicographically ascending
by their raw bytes. Retransmission of a top-level
mission lifecycle object reuses its exact header, sequence, event identity,
and canonical bytes. A nested `SurveyPlanEvidence` retains the exact accepted
mission and baseline-map identities.

Hybrid-map objects use the same canonical algorithm:

- `HybridMapSnapshot`: clear `map_content_identity`;
- `MapAck`: clear `ack_content_identity`.

`HybridMapSnapshot` has no delivery header. All of its source clock, generation
time, synchronized observation time, grid geometry, layer values, dependency
profiles, and semantic regions remain in the business content hash. Semantic regions MUST be unique and lexicographically ascending by `region_id`.
Grid values remain in normative X-fastest row-major order.

Scout navigation snapshots use the same canonical algorithm:

- `ScoutNavigationState`: clear `header` and `navigation_content_identity`.

The delivery header supplies producer session, stream ordering, clock domain,
and audit time but does not alter the navigation business identity. All other
fields, including navigation version, source observation tick, pose, body
twist, both covariance matrices, validity, and timing-profile reference remain
in the hash. At-least-once retransmission within one producer session still
reuses the exact complete bytes, including the header.

Scout sensor and current inputs use the same canonical algorithm:

- `ScoutSensorGeometry`: clear `header` and `geometry_content_identity`;
- `ScoutSensorHealthState`: clear `header` and `health_content_identity`;
- `ScoutCurrentEstimate`: clear `header` and `current_content_identity`.

The delivery header does not alter these business identities. Geometry retains
its calibration provenance, operating domain, fixed extrinsics, visibility
model, and production-approval flag. Health retains its own independent
version, source-domain validity window, state, and unique ascending fault-code
set. Current retains its complete spatial and temporal applicability, local
affine model, complete component and norm error bounds, operating domain,
validity, and model source. An at-least-once retransmission still reuses the
exact complete bytes, including its header.

Scout capability and energy inputs use the same canonical algorithm:

- `ScoutCapabilityProfile`: clear `header` and `capability_content_identity`;
- `ScoutThrusterHealthState`: clear `header` and `health_content_identity`;
- `ScoutEnergyModelProfile`: clear `header` and `energy_model_content_identity`;
- `ScoutEnergyState`: clear `header` and `energy_state_content_identity`.

The capability identity retains its health profile, complete motion, braking
and operating envelopes, allocator version, vehicle/device identity,
calibration provenance and production-approval flag. Thruster health retains
the exact active capability reference, ordered unit states, validity window and
ascending fault codes. The energy-model identity retains its exact capability
reference, conservative coefficients and calibration provenance. Energy state
retains the exact model reference, all plan/return/risk/reserve quantities,
contingency requirement and validity window. The header remains part of exact
delivery identity and at-least-once retransmission bytes.

Main-robot prediction and Scout coordination inputs use the same canonical
algorithm:

- `MainRobotPrediction`: clear `header` and `prediction_content_identity`;
- `ScoutCoordinationConstraint`: clear `header` and
  `coordination_content_identity`.

The prediction identity retains the exact mission tuple, prediction identity
and version, main-robot identity, synchronized alignment epoch, sender-domain
validity bounds, ordered closed occupied intervals, swept centers, physical
radius, uncertainty radius and conservative occupied radius. The coordination
identity retains the exact mission and prediction references, coordination
version, `MAIN_SCOUT_COOP` channel, separation and communication limits,
link-assurance basis, optional calibrated-model reference, both directional
LossPolicy references and sender-domain validity bounds. The delivery header
does not alter either business identity. It remains part of exact delivery
identity and byte-identical retransmission checks.

Scout 4D planning objects use the same canonical algorithm:

- `ScoutTrajectory4d`: clear `trajectory_content_identity`;
- `ScoutPlanningDependencies`: clear `dependencies_content_identity`;
- `ScoutPlanValidationReport`: clear `validation_report_content_identity`;
- `ScoutPlan`: clear `plan_content_identity`;
- `ScoutPlanningResult`: clear `header` and `result_content_identity`.

The trajectory identity retains the normalized `initial_yaw_rad`, ordered
segments, contiguous start offsets, positive durations, all six three-dimensional
control points per segment, and all six unwrapped yaw offsets per segment. Yaw
offsets are rotation displacements and MUST NOT be normalized modulo `2*pi`.
The dependency identity retains every exact Ticket 02-07 input version and
content identity plus the planner-configuration, `TimingProfile`, and
`InterfaceLimits` references. Sensor dependencies
MUST be unique and lexicographically ascending by `sensor_id`.

The validation-report identity retains its status, primary outcome, optional
failure offset, margins, refinement depth, structured diagnostics, and exact
validated trajectory/dependency/survey-evidence identities. The plan identity
retains the populated nested trajectory, dependencies, `SurveyPlanEvidence`,
and independent validation-report identities. The result business identity
retains its sequence, outcome, optional candidate, evaluation time, dependency
summary, and diagnostics. Its delivery header remains outside business
identity but remains part of byte-identical retransmission behavior.

Scout authorization objects use the same canonical algorithm:

- `ScoutExecutionLease`: clear `content_identity`;
- `ScoutAuthorizedExecutionBundle`: clear `bundle_content_identity`;
- `ScoutBundleAck`: has no independent business identity and retains the exact
  observed Bundle identity.

The Scout lease identity retains the exact Scout plan sequence and identity,
validation and expiry ticks, authorized trajectory-time interval, and complete
planning dependencies. The Scout Bundle identity retains its delivery header,
strong Scout-motion domain marker, exact immutable plan and lease, fixed
execution epoch, installation window, TimingProfile, InterfaceLimits, and
SafetyGate configuration references. A retransmission reuses the exact bytes.
The ACK is accepted only when its Scout stream, authority clock domain, Bundle,
plan, lease, and observed Bundle identity match the installed object.

Scout execution feedback and revocation use the same canonical validation
rules with these identities:

- `ScoutExecutionFeedback` has no independent business identity. Its exact
  delivery header, installed Bundle/plan/trajectory/lease identities, fixed
  execution offset, three target/state views, control and safety state, and FCU
  session are validated together. Feedback is latest-value evidence and never
  grants or renews authority by itself;
- `ScoutExecutionRevocation`: clear `revocation_content_identity`; and
- `ScoutExecutionRevocationAck` has no independent business identity and
  retains the exact observed revocation identity.

The Scout revocation identity retains its delivery header, strong Scout-motion
domain marker, exact authorization identities, stop level, risk action,
structured reason, registered codes, and effective tick. An idempotent retry
reuses the exact bytes. The ACK is accepted only when its adapter and FCU
sessions, stream, authority clock domain, revocation and authorization
sequences, observed revocation identity, application state, outcome, and tick
match. Feedback or an ACK received after revocation cannot alter the persisted
revoked watermark.

Scout state and configuration artifacts use these identity rules:

- `StateTransition`: clear `header` and `transition_content_identity` before
  computing the business identity. Retain the exact domain, previous/next
  numeric states, stable `CodeRef trigger`, state version, entry tick and
  optional channel identity;
- `ScoutConfigurationProfile`, `ScoutCommunicationLossPolicy`,
  `TimingProfile`, and `InterfaceLimits` are immutable profile artifacts. Their
  `ProfileRef.content_identity` is SHA-256 over the exact canonical published
  artifact bytes, including every nested exact dependency reference and every
  explicit optional boolean presence;
- `scout-state-transitions/v1` is a repository artifact whose Manifest identity
  is SHA-256 over its exact UTF-8 file bytes.

Unknown triggers, unknown safety enums, missing presence, unordered or
duplicate profile references, and any nested identity mismatch are rejected
before a state or profile watermark is installed. `SafetyAuditEvent` retains
the rejected numeric value and stable CodeRefs; it is audit evidence and never
participates in authorization identity.

For `MapAck`, exclude the top-level `MessageHeader` from the business content
hash and retain the exact uncompressed map identity. Missing chunk indexes MUST
be unique and strictly ascending. Map chunks are transfer envelopes and are not included in the map business content hash.
The `uncompressed_map_identity` carried by every chunk instead hashes the exact
deterministic serialized `HybridMapSnapshot` bytes after its populated business
content identity is installed. Per-chunk identities hash exact payload bytes.

Adapters MUST compare the locally recomputed 32 bytes in constant time before
installation. A mismatch rejects the whole object and raises
`FAULT_BUNDLE_INTEGRITY`.

## Repository artifacts

ContractManifest repository hashes are SHA-256 over exact file bytes. The
Protobuf descriptor hash is over the deterministic descriptor set produced
from all sorted v1 `.proto` inputs with imports included. The CAN combined
identity is SHA-256 over exact DBC bytes followed immediately by exact protocol
supplement bytes.

Changing line endings or encoding therefore changes an artifact identity and
requires a deliberate Manifest update.
