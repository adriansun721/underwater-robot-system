# Scout Planner

ROS-independent C++ planning core for the leading underwater robot. The current
implementation provides a strict build boundary, deterministic three-dimensional
fixtures, and lossless adapters between the approved v1 Protobuf contracts and
validated pure C++ core values. It also captures those values and their activated
configuration as one immutable, dependency-exact planning context and queries its
five-layer hybrid map conservatively. It does not claim production capability or
execution authorization.

## Verify

On Windows with Visual Studio C++ Build Tools installed:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/verify.ps1
```

On a Unix-like host with CMake, Ninja, ICU, Protobuf, and a supported C++
compiler:

```sh
./tools/verify.sh
```

The command configures the project, builds with warnings as errors, runs the
platform static analyzer, executes CTest, and writes JUnit XML to
`build/verify/ctest.xml`.

The build generates C++ from the sorted authoritative schemas under
`../interfaces/proto` and checks the complete v1 descriptor against the Contract
Manifest SHA-256. `RelWithDebInfo` is used so the generated code and the installed
release Protobuf runtime share one MSVC CRT ABI while retaining debug symbols.

## Protobuf/core boundary

`scout_planner/core/protobuf_adapter.hpp` exposes tagged, non-default-constructible
core values for mission, navigation, map, current, sensor, capability, energy,
coordination, trajectory, validation, planning, and Scout authorization data.
`ProtobufAdapter` is the only conversion seam. It preserves every populated public
field and fails closed for unknown or missing safety data, invalid numbers, frames,
times, versions, frame-specific covariance, structure, activated collection and
message-size limits, or canonical content
identity. Identity verification is mandatory and recursive, including identities
embedded in plans and authorized bundles. Producer code can use
`canonicalize_and_identify` to validate, normalize, and rebuild the complete
identity tree from its leaves before deterministic serialization.

## Atomic planning context

`scout_planner/core/planning_context.hpp` exposes
`PlanningContextBuilder::capture`. The builder freezes a mission together with
its identity-verified, explicitly accepted `ScoutMissionDecision`,
navigation, map,
current, paired sensor geometry/health, capability, thruster health, energy,
main-robot prediction, coordination, exact planning dependencies, local receipt
times, and the activated configuration. It rejects capture races, stale or future
local facts, clock/synchronization mismatches, cross-mission or cross-prediction
mixes, operating-domain drift, dependency identity drift, competing canonical
publishers, delivery reorder (including multiple logical sensors on one stream),
business-version rollback, and retired-session replay. A successful
`ScoutPlanningContext` only provides const access to the copied snapshot; planning
code does not read latest caches through this interface.

## Hybrid map query

`scout_planner/core/hybrid_map_query.hpp` exposes the Ticket 16 point and
deterministic segment-supercover seam. Construction requires the exact map ID,
version, and canonical content identity bound by planning dependencies. Every
query combines the immutable snapshot's seafloor, 3D occupancy, ESDF, allowed
water, declared map region, and semantic AABBs. It returns the effective
FREE/OCCUPIED/UNKNOWN/STALE/CONFLICTED state, conservative clearance, seafloor
quality, source versions/timestamps/region, semantic restrictions, queried
voxels, and located information gaps. Body, localization, tracking, map, and
discretization margins remain separate finite non-negative SI inputs; their sum
conservatively expands allowed-water cells and semantic AABBs as well as the
ESDF, seafloor, and map-boundary checks. Boundaries and uncertain data fail
closed; no query result grants execution authorization.

## Capability and energy gate

`scout_planner/core/capability_energy_gate.hpp` exposes the Ticket 17 hard
gate. It evaluates finite trajectory samples against the immutable context's
validated capability, thruster-health, current, and energy snapshots. Current
velocity is transformed by the sampled ENU yaw into the body-relative frame and
evaluated in the `mission_enu` applicability region with its declared error
bound; when no spatial gradient is present, callers must provide
explicit non-negative calibrated acceleration and yaw error margins. Capability
or energy failure is returned as a structured result with the first failing
sample and audited versions. Plan energy includes model error, contingency
energy (return or risk action), and reserve. Domain, validity, health-profile,
production-approval, non-finite, and out-of-region mismatches fail closed.
The implementation is an algorithm seam only and does not claim production
calibration, thruster allocation proof, or execution authorization.

## Bootstrap fixture boundary

`scout_planner/testing/deterministic_fixture.hpp` exposes the Ticket 13 test
fixture seam. It includes flat seabed, overhang, narrow passage, UNKNOWN,
STALE, CONFLICTED, and moving-main-robot scenarios. Moving occupancy uses
contiguous closed intervals of conservative swept spheres with explicit physical
and position-uncertainty radii. Every fixture is marked non-production and
carries its seed, SI units, `mission_enu` frame, local clock domain, and ordered
input versions into failure output.

## Deterministic state-lattice search

`scout_planner/core/state_lattice_astar.hpp` exposes the ROS-independent
`TimeAwareStateLatticeAStar3d` seam. It searches voxel, heading-bin, and action
mode states with fixed 26-neighbour primitives. Each edge is checked through
the hybrid map's continuous supercover query; unknown, stale, conflicted,
occupied, insufficient-clearance, and disallowed-water cells are rejected.
Non-negative distance/time costs, a goal-AABB speed lower-bound heuristic, and
bounded node/queue/memory/deadline/cancellation controls make replay and failure
outcomes deterministic. Results are geometric seeds only and do not grant
execution authority.

## Time-aware cooperative search

The state-lattice seam supports a bounded number of Scout-local arrival-time
labels, optional WAIT transitions, and conservative cooperative constraints.
`CooperativeSearchConstraint` models contiguous moving main-robot swept spheres;
each edge is sampled at its start, midpoint, and end for separation and
communication bounds. `SearchEnergyBudget` provides a lower-bound power budget
including return and reserve energy for early pruning. Survey-mode results carry
an `StateLatticeActionSeed` with non-empty approach, observe, and exit vectors;
an optional exit region keeps the search running after observation. These are
planning seeds only and require independent smoothing, validation, and safety
authorization before execution.
