"""Public-contract tests for immutable hybrid 3D map snapshots."""

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
import zlib


INTERFACES = pathlib.Path(__file__).resolve().parents[1]
PROTO_ROOT = INTERFACES / "proto"
PROTO_V1 = PROTO_ROOT / "underwater" / "contracts" / "v1"
CONTRACT_PATH = INTERFACES / "HYBRID_MAP_SNAPSHOT.md"
PROFILE_PATH = INTERFACES / "profiles" / "integration-v1.json"
MANIFEST_PATH = INTERFACES / "compatibility" / "contract-manifest-v1.json"


def canonical_snapshot_identity(snapshot: object) -> bytes:
    canonical = type(snapshot)()
    canonical.CopyFrom(snapshot)
    canonical.ClearField("map_content_identity")
    return hashlib.sha256(
        canonical.SerializeToString(deterministic=True)
    ).digest()


def canonical_map_ack_identity(acknowledgement: object) -> bytes:
    canonical = type(acknowledgement)()
    canonical.CopyFrom(acknowledgement)
    canonical.ClearField("header")
    canonical.ClearField("ack_content_identity")
    return hashlib.sha256(
        canonical.SerializeToString(deterministic=True)
    ).digest()


def _validate_region(region: object) -> None:
    values = list(region.xyz_m)
    if (
        region.frame_id != "mission_enu"
        or len(values) != 6
        or not all(math.isfinite(value) for value in values)
        or not all(values[index] < values[index + 3] for index in range(3))
    ):
        raise ValueError("invalid ENU region")


def _validate_profile_ref(profile: object) -> None:
    if (
        not profile.profile_id
        or profile.version == 0
        or len(profile.content_identity.sha256) != 32
    ):
        raise ValueError("invalid profile reference")


def _grid_2d_key(grid: object) -> tuple[object, ...]:
    scalar_fields = ("origin_x_m", "origin_y_m", "resolution_x_m", "resolution_y_m")
    if not all(grid.HasField(field) for field in scalar_fields):
        raise ValueError("missing 2D grid scalar")
    scalars = tuple(getattr(grid, field) for field in scalar_fields)
    if (
        not all(math.isfinite(value) for value in scalars)
        or grid.resolution_x_m <= 0.0
        or grid.resolution_y_m <= 0.0
        or grid.cell_count_x == 0
        or grid.cell_count_y == 0
        or grid.frame_id != "mission_enu"
    ):
        raise ValueError("invalid 2D grid")
    return (*scalars, grid.cell_count_x, grid.cell_count_y, grid.frame_id)


def _grid_3d_key(grid: object) -> tuple[object, ...]:
    scalar_fields = (
        "origin_x_m",
        "origin_y_m",
        "origin_z_m",
        "resolution_x_m",
        "resolution_y_m",
        "resolution_z_m",
    )
    if not all(grid.HasField(field) for field in scalar_fields):
        raise ValueError("missing 3D grid scalar")
    scalars = tuple(getattr(grid, field) for field in scalar_fields)
    if (
        not all(math.isfinite(value) for value in scalars)
        or min(grid.resolution_x_m, grid.resolution_y_m, grid.resolution_z_m) <= 0.0
        or min(grid.cell_count_x, grid.cell_count_y, grid.cell_count_z) == 0
        or grid.frame_id != "mission_enu"
    ):
        raise ValueError("invalid 3D grid")
    return (
        *scalars,
        grid.cell_count_x,
        grid.cell_count_y,
        grid.cell_count_z,
        grid.frame_id,
    )


def validate_snapshot(
    snapshot: object,
    mapping: object,
    common: object,
    limits: dict[str, int] | None = None,
) -> None:
    """Executable consumer gate for the public conformance vectors."""
    if (
        not snapshot.source_clock_domain_id
        or not snapshot.map_id
        or snapshot.map_version == 0
        or len(snapshot.map_content_identity.sha256) != 32
        or not snapshot.HasField("generated_at_monotonic_ns")
        or snapshot.generated_at_monotonic_ns < 0
        or not snapshot.HasField("observed_at")
        or not snapshot.observed_at.HasField("utc_time_ns")
        or not snapshot.observed_at.HasField("uncertainty_ns")
        or snapshot.observed_at.status
        not in {
            common.TIME_SYNC_UNSYNCHRONIZED,
            common.TIME_SYNC_SYNCHRONIZED,
            common.TIME_SYNC_DEGRADED,
        }
    ):
        raise ValueError("invalid snapshot identity or time")
    if not hmac.compare_digest(
        snapshot.map_content_identity.sha256,
        canonical_snapshot_identity(snapshot),
    ):
        raise ValueError("map content identity mismatch")
    _validate_region(snapshot.map_region)
    _validate_profile_ref(snapshot.sensor_extrinsic)
    _validate_profile_ref(snapshot.mapping_parameters)
    for layer in ("seafloor", "occupancy", "esdf", "allowed_water"):
        if not snapshot.HasField(layer):
            raise ValueError(f"missing required layer: {layer}")

    grid_2d = _grid_2d_key(snapshot.seafloor.grid)
    grid_3d = _grid_3d_key(snapshot.occupancy.grid)
    if _grid_3d_key(snapshot.esdf.grid) != grid_3d:
        raise ValueError("ESDF grid mismatch")
    if _grid_3d_key(snapshot.allowed_water.grid) != grid_3d:
        raise ValueError("allowed-water grid mismatch")
    if (
        grid_2d[0] != grid_3d[0]
        or grid_2d[1] != grid_3d[1]
        or grid_2d[2] != grid_3d[3]
        or grid_2d[3] != grid_3d[4]
        or grid_2d[4] != grid_3d[6]
        or grid_2d[5] != grid_3d[7]
    ):
        raise ValueError("seafloor grid mismatch")

    cells_2d = snapshot.seafloor.grid.cell_count_x * snapshot.seafloor.grid.cell_count_y
    cells_3d = (
        snapshot.occupancy.grid.cell_count_x
        * snapshot.occupancy.grid.cell_count_y
        * snapshot.occupancy.grid.cell_count_z
    )
    if limits is not None and (
        cells_2d > limits["maximum_map_cells_per_layer"]
        or cells_3d > limits["maximum_map_cells_per_layer"]
        or len(snapshot.semantic_regions) > limits["maximum_map_semantic_regions"]
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED")
    if len(snapshot.seafloor.elevation_z_m) != cells_2d or len(snapshot.seafloor.quality) != cells_2d:
        raise ValueError("seafloor layer length mismatch")
    if not all(math.isfinite(value) for value in snapshot.seafloor.elevation_z_m):
        raise ValueError("invalid seafloor elevation")
    if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in snapshot.seafloor.quality):
        raise ValueError("invalid seafloor quality")
    known_voxel_states = {
        value.number
        for value in mapping.VoxelState.DESCRIPTOR.values
        if value.number != 0
    }
    if len(snapshot.occupancy.state) != cells_3d:
        raise ValueError("occupancy layer length mismatch")
    if any(state not in known_voxel_states for state in snapshot.occupancy.state):
        raise ValueError("unknown voxel state")
    known_esdf_conventions = {
        value.number
        for value in mapping.EsdfConvention.DESCRIPTOR.values
        if value.number != 0
    }
    if (
        snapshot.esdf.convention not in known_esdf_conventions
        or len(snapshot.esdf.distance_m) != cells_3d
        or not all(math.isfinite(value) for value in snapshot.esdf.distance_m)
    ):
        raise ValueError("invalid ESDF layer")
    if len(snapshot.allowed_water.allowed) != cells_3d:
        raise ValueError("allowed-water layer length mismatch")

    known_semantic_classes = {
        value.number
        for value in mapping.SemanticClass.DESCRIPTOR.values
        if value.number != 0
    }
    region_ids = [region.region_id for region in snapshot.semantic_regions]
    if region_ids != sorted(set(region_ids)):
        raise ValueError("semantic region ordering conflict")
    for region in snapshot.semantic_regions:
        if not region.region_id or region.semantic_class not in known_semantic_classes:
            raise ValueError("unknown semantic class")
        _validate_region(region.region)


def make_map_chunks(
    snapshot: object, cooperation: object, common: object, chunk_count: int = 2
) -> list[object]:
    payload = snapshot.SerializeToString(deterministic=True)
    split_points = [len(payload) * index // chunk_count for index in range(chunk_count + 1)]
    parts = [payload[split_points[index] : split_points[index + 1]] for index in range(chunk_count)]
    chunks = []
    offset = 0
    for index, part in enumerate(parts):
        chunks.append(
            cooperation.MapChunk(
                map_id=snapshot.map_id,
                map_version=snapshot.map_version,
                frame_id="mission_enu",
                chunk_index=index,
                chunk_count=chunk_count,
                compressed_payload=part,
                uncompressed_map_identity=common.ContentIdentity(
                    sha256=hashlib.sha256(payload).digest()
                ),
                uncompressed_crc32=zlib.crc32(payload) & 0xFFFFFFFF,
                payload_encoding=cooperation.MAP_PAYLOAD_DETERMINISTIC_PROTOBUF_V1,
                compression=cooperation.MAP_COMPRESSION_NONE,
                compressed_total_bytes=len(payload),
                uncompressed_total_bytes=len(payload),
                compressed_offset_bytes=offset,
                compressed_chunk_crc32=zlib.crc32(part) & 0xFFFFFFFF,
                compressed_chunk_identity=common.ContentIdentity(
                    sha256=hashlib.sha256(part).digest()
                ),
            )
        )
        offset += len(part)
    return chunks


def assemble_uncompressed_map(
    chunks: list[object], cooperation: object, mapping: object, common: object,
    limits: dict[str, int] | None = None,
) -> bytes:
    """Independent consumer for the uncompressed conformance vector."""
    if not chunks:
        raise ValueError("missing chunks")
    first = chunks[0]
    if (
        first.payload_encoding != cooperation.MAP_PAYLOAD_DETERMINISTIC_PROTOBUF_V1
        or first.compression != cooperation.MAP_COMPRESSION_NONE
    ):
        raise ValueError("unsupported encoding or compression")
    if limits is not None and (
        first.chunk_count > limits["maximum_map_chunks_per_snapshot"]
        or first.compressed_total_bytes > limits["maximum_map_snapshot_bytes"]
        or first.uncompressed_total_bytes > limits["maximum_map_snapshot_bytes"]
        or any(
            len(chunk.compressed_payload) > limits["maximum_map_chunk_bytes"]
            for chunk in chunks
        )
    ):
        raise ValueError("RESOURCE_LIMIT_EXCEEDED")

    metadata = (
        first.map_id,
        first.map_version,
        first.frame_id,
        first.chunk_count,
        first.payload_encoding,
        first.compression,
        first.compressed_total_bytes,
        first.uncompressed_total_bytes,
        first.uncompressed_map_identity.sha256,
        first.uncompressed_crc32,
    )
    by_index: dict[int, object] = {}
    for chunk in chunks:
        chunk_metadata = (
            chunk.map_id,
            chunk.map_version,
            chunk.frame_id,
            chunk.chunk_count,
            chunk.payload_encoding,
            chunk.compression,
            chunk.compressed_total_bytes,
            chunk.uncompressed_total_bytes,
            chunk.uncompressed_map_identity.sha256,
            chunk.uncompressed_crc32,
        )
        if chunk_metadata != metadata:
            raise ValueError("mixed transfer metadata")
        previous = by_index.get(chunk.chunk_index)
        if previous is not None:
            previous_business = type(previous)()
            previous_business.CopyFrom(previous)
            previous_business.ClearField("header")
            chunk_business = type(chunk)()
            chunk_business.CopyFrom(chunk)
            chunk_business.ClearField("header")
            if previous_business != chunk_business:
                raise ValueError("conflicting duplicate")
            continue
        by_index[chunk.chunk_index] = chunk

    if set(by_index) != set(range(first.chunk_count)):
        raise ValueError("missing chunks")
    ordered = [by_index[index] for index in range(first.chunk_count)]
    offset = 0
    for chunk in ordered:
        if chunk.compressed_offset_bytes != offset:
            raise ValueError("non-contiguous chunk offset")
        if zlib.crc32(chunk.compressed_payload) & 0xFFFFFFFF != chunk.compressed_chunk_crc32:
            raise ValueError("chunk CRC mismatch")
        if hashlib.sha256(chunk.compressed_payload).digest() != chunk.compressed_chunk_identity.sha256:
            raise ValueError("chunk identity mismatch")
        offset += len(chunk.compressed_payload)
    compressed = b"".join(chunk.compressed_payload for chunk in ordered)
    if len(compressed) != first.compressed_total_bytes:
        raise ValueError("compressed size mismatch")
    payload = compressed
    if len(payload) != first.uncompressed_total_bytes:
        raise ValueError("uncompressed size mismatch")
    if zlib.crc32(payload) & 0xFFFFFFFF != first.uncompressed_crc32:
        raise ValueError("map CRC mismatch")
    if hashlib.sha256(payload).digest() != first.uncompressed_map_identity.sha256:
        raise ValueError("map identity mismatch")
    snapshot = mapping.HybridMapSnapshot.FromString(payload)
    without_unknown = mapping.HybridMapSnapshot()
    without_unknown.CopyFrom(snapshot)
    without_unknown.DiscardUnknownFields()
    if without_unknown.SerializeToString(deterministic=True) != payload:
        raise ValueError("unknown snapshot fields")
    validate_snapshot(snapshot, mapping, common, limits)
    return payload


def apply_map_watermark(
    current_version: int,
    current_identity: bytes,
    incoming_version: int,
    incoming_identity: bytes,
) -> str:
    if incoming_version < current_version:
        raise ValueError("VERSION_INCOMPATIBLE")
    if incoming_version == current_version and incoming_identity != current_identity:
        raise ValueError("INPUT_INVALID")
    if incoming_version == current_version:
        return "idempotent duplicate"
    return "accepted"


class HybridMapSnapshotContract(unittest.TestCase):
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
        cls.mapping = importlib.import_module("underwater.contracts.v1.mapping_pb2")

    @classmethod
    def tearDownClass(cls) -> None:
        sys.path.remove(cls.generated.name)
        cls.generated.cleanup()

    def _complete_snapshot(self) -> object:
        grid_3d = self.mapping.GridGeometry3d(
            origin_x_m=0.0,
            origin_y_m=0.0,
            origin_z_m=-4.0,
            resolution_x_m=1.0,
            resolution_y_m=1.0,
            resolution_z_m=1.0,
            cell_count_x=2,
            cell_count_y=2,
            cell_count_z=4,
            frame_id="mission_enu",
        )
        snapshot = self.mapping.HybridMapSnapshot(
            source_clock_domain_id="mapping-nuc/boot-8",
            map_id="scout-local-map",
            map_version=17,
            map_content_identity=self.common.ContentIdentity(sha256=b"m" * 32),
            map_region=self.cooperation.Region3dEnu(
                xyz_m=[0.0, 0.0, -4.0, 2.0, 2.0, 0.0],
                frame_id="mission_enu",
            ),
            sensor_extrinsic=self.common.ProfileRef(
                profile_id="scout-sensors/extrinsics",
                version=4,
                content_identity=self.common.ContentIdentity(sha256=b"e" * 32),
            ),
            mapping_parameters=self.common.ProfileRef(
                profile_id="scout-mapping/parameters",
                version=9,
                content_identity=self.common.ContentIdentity(sha256=b"p" * 32),
            ),
            generated_at_monotonic_ns=8_000_000_000,
            observed_at=self.common.SynchronizedObservationTime(
                utc_time_ns=1_800_000_000_000_000_000,
                status=self.common.TIME_SYNC_SYNCHRONIZED,
                uncertainty_ns=2_000_000,
            ),
            seafloor=self.mapping.SeafloorElevationLayer(
                grid=self.mapping.GridGeometry2d(
                    origin_x_m=0.0,
                    origin_y_m=0.0,
                    resolution_x_m=1.0,
                    resolution_y_m=1.0,
                    cell_count_x=2,
                    cell_count_y=2,
                    frame_id="mission_enu",
                ),
                elevation_z_m=[-3.0, -3.1, -3.2, -3.3],
                quality=[1.0, 0.9, 0.8, 0.7],
            ),
            occupancy=self.mapping.VoxelOccupancyLayer(
                grid=grid_3d,
                state=[
                    self.mapping.VOXEL_FREE,
                    self.mapping.VOXEL_OCCUPIED,
                    self.mapping.VOXEL_UNKNOWN,
                    self.mapping.VOXEL_STALE,
                    self.mapping.VOXEL_CONFLICTED,
                    *([self.mapping.VOXEL_FREE] * 11),
                ],
            ),
            esdf=self.mapping.EsdfLayer(
                grid=grid_3d,
                convention=self.mapping.ESDF_SIGNED_DISTANCE,
                distance_m=[0.5, -0.25, *([1.0] * 14)],
            ),
            allowed_water=self.mapping.AllowedWaterLayer(
                grid=grid_3d,
                allowed=[True, False, *([True] * 14)],
            ),
            semantic_regions=[
                self.mapping.SemanticRegion(
                    region_id="overhang-1",
                    semantic_class=self.mapping.SEMANTIC_NO_ENTRY,
                    region=self.cooperation.Region3dEnu(
                        xyz_m=[0.5, 0.5, -2.0, 1.5, 1.5, -1.0],
                        frame_id="mission_enu",
                    ),
                )
            ],
        )
        snapshot.map_content_identity.sha256 = canonical_snapshot_identity(snapshot)

        return snapshot

    def test_complete_hybrid_snapshot_round_trip_preserves_all_layers(self) -> None:
        snapshot = self._complete_snapshot()
        decoded = self.mapping.HybridMapSnapshot.FromString(
            snapshot.SerializeToString(deterministic=True)
        )

        validate_snapshot(decoded, self.mapping, self.common)
        self.assertEqual(decoded, snapshot)
        self.assertEqual(decoded.map_region.frame_id, "mission_enu")
        self.assertEqual(decoded.sensor_extrinsic.version, 4)
        self.assertEqual(decoded.mapping_parameters.version, 9)
        self.assertEqual(decoded.occupancy.state[1], self.mapping.VOXEL_OCCUPIED)
        self.assertEqual(decoded.semantic_regions[0].region_id, "overhang-1")

    def test_unknown_voxel_state_is_preserved_on_wire_and_rejected_by_contract(self) -> None:
        snapshot = self._complete_snapshot()
        snapshot.occupancy.state[0] = 99
        snapshot.map_content_identity.sha256 = canonical_snapshot_identity(snapshot)
        decoded = self.mapping.HybridMapSnapshot.FromString(
            snapshot.SerializeToString(deterministic=True)
        )
        known_states = {
            value.number
            for value in self.mapping.VoxelState.DESCRIPTOR.values
            if value.number != 0
        }

        self.assertEqual(decoded.occupancy.state[0], 99)
        self.assertNotIn(decoded.occupancy.state[0], known_states)
        with self.assertRaisesRegex(ValueError, "unknown voxel state"):
            validate_snapshot(decoded, self.mapping, self.common)
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        self.assertIn("unknown voxel state | reject the whole snapshot", contract)
        self.assertIn(
            "A seafloor elevation cell MUST NOT imply that the water column above it is free",
            contract,
        )

    def test_complete_reassembly_accepts_out_of_order_identical_duplicate(self) -> None:
        snapshot = self._complete_snapshot()
        payload = snapshot.SerializeToString(deterministic=True)
        chunks = make_map_chunks(snapshot, self.cooperation, self.common)

        rebuilt = assemble_uncompressed_map(
            [chunks[1], chunks[0], chunks[1]],
            self.cooperation,
            self.mapping,
            self.common,
        )
        decoded = self.mapping.HybridMapSnapshot.FromString(rebuilt)

        self.assertEqual(rebuilt, payload)
        self.assertEqual(decoded, snapshot)

        mixed = self.cooperation.MapChunk()
        mixed.CopyFrom(chunks[1])
        mixed.frame_id = "other_frame"
        with self.assertRaisesRegex(ValueError, "mixed transfer metadata"):
            assemble_uncompressed_map(
                [chunks[0], mixed],
                self.cooperation,
                self.mapping,
                self.common,
            )

    def test_missing_chunks_are_requested_by_identity_without_partial_publish(self) -> None:
        acknowledgement = self.cooperation.MapAck(
            header=self.common.MessageHeader(
                stream_id=self.common.STREAM_MAP_ACK,
                sequence=12,
                source_clock_domain_id="scout-planner/boot-2",
            ),
            map_id="scout-local-map",
            map_version=17,
            disposition=self.cooperation.MAP_ACK_MISSING_CHUNKS,
            missing_chunk_indexes=[1, 3],
            uncompressed_map_identity=self.common.ContentIdentity(sha256=b"u" * 32),
            expected_chunk_count=4,
            acknowledgement_version=2,
            outcome=self.common.CodeRef(
                numeric_code=2,
                registry_id="underwater-system-codes",
                registry_version=1,
            ),
            ack_content_identity=self.common.ContentIdentity(sha256=b"a" * 32),
        )
        acknowledgement.ack_content_identity.sha256 = canonical_map_ack_identity(
            acknowledgement
        )

        decoded = self.cooperation.MapAck.FromString(
            acknowledgement.SerializeToString(deterministic=True)
        )
        contract = CONTRACT_PATH.read_text(encoding="utf-8")

        self.assertEqual(decoded, acknowledgement)
        self.assertTrue(
            hmac.compare_digest(
                decoded.ack_content_identity.sha256,
                canonical_map_ack_identity(decoded),
            )
        )
        self.assertEqual(list(decoded.missing_chunk_indexes), [1, 3])
        self.assertEqual(decoded.uncompressed_map_identity.sha256, b"u" * 32)
        self.assertIn("No layer or partial snapshot is published while chunks are missing", contract)
        self.assertIn("missing indexes MUST be unique and strictly ascending", contract)

        incomplete = make_map_chunks(
            self._complete_snapshot(), self.cooperation, self.common
        )[0]
        with self.assertRaisesRegex(ValueError, "missing chunks"):
            assemble_uncompressed_map(
                [incomplete], self.cooperation, self.mapping, self.common
            )

    def test_overhanging_obstacle_is_independent_from_valid_seafloor(self) -> None:
        snapshot = self._complete_snapshot()
        snapshot.occupancy.state[:] = [self.mapping.VOXEL_FREE] * 16
        snapshot.occupancy.state[12] = self.mapping.VOXEL_OCCUPIED
        snapshot.map_content_identity.sha256 = canonical_snapshot_identity(snapshot)

        decoded = self.mapping.HybridMapSnapshot.FromString(
            snapshot.SerializeToString(deterministic=True)
        )

        validate_snapshot(decoded, self.mapping, self.common)
        self.assertEqual(decoded.seafloor.elevation_z_m[0], -3.0)
        self.assertEqual(decoded.occupancy.state[12], self.mapping.VOXEL_OCCUPIED)
        self.assertIn(
            "Suspended and overhanging obstacles are represented independently",
            CONTRACT_PATH.read_text(encoding="utf-8"),
        )

    def test_interface_limits_bound_whole_snapshot_chunks_cells_and_regions(self) -> None:
        profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        limits = profile["interface_limits"]
        schema = (PROTO_V1 / "profiles.proto").read_text(encoding="utf-8")

        for name in (
            "maximum_map_snapshot_bytes",
            "maximum_map_chunks_per_snapshot",
            "maximum_map_cells_per_layer",
            "maximum_map_semantic_regions",
        ):
            self.assertIn(name, limits)
            self.assertGreater(limits[name], 0)
            self.assertIn(name, schema)
        self.assertLessEqual(
            limits["maximum_map_chunk_bytes"],
            limits["maximum_map_snapshot_bytes"],
        )
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        self.assertIn("RESOURCE_LIMIT_EXCEEDED", contract)
        self.assertIn("MUST NOT truncate", contract)

        chunks = make_map_chunks(
            self._complete_snapshot(), self.cooperation, self.common
        )
        too_small = dict(limits)
        too_small["maximum_map_snapshot_bytes"] = 1
        with self.assertRaisesRegex(ValueError, "RESOURCE_LIMIT_EXCEEDED"):
            assemble_uncompressed_map(
                chunks, self.cooperation, self.mapping, self.common, too_small
            )
        too_few_cells = dict(limits)
        too_few_cells["maximum_map_cells_per_layer"] = 15
        with self.assertRaisesRegex(ValueError, "RESOURCE_LIMIT_EXCEEDED"):
            assemble_uncompressed_map(
                chunks, self.cooperation, self.mapping, self.common, too_few_cells
            )

    def test_crc_and_hash_errors_reject_the_transfer(self) -> None:
        valid = make_map_chunks(
            self._complete_snapshot(), self.cooperation, self.common, chunk_count=1
        )[0]
        bad_crc = self.cooperation.MapChunk()
        bad_crc.CopyFrom(valid)
        bad_crc.compressed_chunk_crc32 ^= 1
        bad_hash = self.cooperation.MapChunk()
        bad_hash.CopyFrom(valid)
        bad_hash.uncompressed_map_identity.sha256 = b"x" * 32

        with self.assertRaisesRegex(ValueError, "chunk CRC mismatch"):
            assemble_uncompressed_map(
                [bad_crc], self.cooperation, self.mapping, self.common
            )
        with self.assertRaisesRegex(ValueError, "map identity mismatch"):
            assemble_uncompressed_map(
                [bad_hash], self.cooperation, self.mapping, self.common
            )

    def test_version_rollback_and_conflicting_identity_are_fail_closed(self) -> None:
        contract = CONTRACT_PATH.read_text(encoding="utf-8")
        for rule in (
            "version equality with different content identity is an integrity conflict",
            "A lower version is a version rollback",
            "version rollback | `VERSION_INCOMPATIBLE`",
            "same map version + different content identity | `INPUT_INVALID`",
        ):
            self.assertIn(rule, contract)
        current_identity = b"c" * 32
        self.assertEqual(
            apply_map_watermark(17, current_identity, 17, current_identity),
            "idempotent duplicate",
        )
        self.assertEqual(
            apply_map_watermark(17, current_identity, 18, b"n" * 32),
            "accepted",
        )
        with self.assertRaisesRegex(ValueError, "VERSION_INCOMPATIBLE"):
            apply_map_watermark(17, current_identity, 16, b"o" * 32)
        with self.assertRaisesRegex(ValueError, "INPUT_INVALID"):
            apply_map_watermark(17, current_identity, 17, b"x" * 32)

    def test_hashing_rules_cover_snapshot_and_ack_identities(self) -> None:
        hashing = (INTERFACES / "HASHING.md").read_text(encoding="utf-8")
        for rule in (
            "`HybridMapSnapshot`: clear `map_content_identity`",
            "`MapAck`: clear `ack_content_identity`",
            "Semantic regions MUST be unique and lexicographically ascending by `region_id`",
            "Map chunks are transfer envelopes and are not included in the map business content hash",
        ):
            self.assertIn(rule, hashing)

        snapshot = self._complete_snapshot()
        self.assertTrue(
            hmac.compare_digest(
                snapshot.map_content_identity.sha256,
                canonical_snapshot_identity(snapshot),
            )
        )
        tampered = self.mapping.HybridMapSnapshot()
        tampered.CopyFrom(snapshot)
        tampered.map_version += 1
        with self.assertRaisesRegex(ValueError, "map content identity mismatch"):
            validate_snapshot(tampered, self.mapping, self.common)

    def test_manifest_gates_hybrid_map_snapshot_as_exact_feature(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))

        self.assertIn("scout_hybrid_map_snapshot_v1", manifest["supported_features"])
        self.assertEqual(manifest["approved_mixed_versions"], [])


if __name__ == "__main__":
    unittest.main()
